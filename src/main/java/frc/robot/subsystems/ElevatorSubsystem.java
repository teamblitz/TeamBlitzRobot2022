package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.StatusManager;

/**
 * Welcome to this mess I have been working on.
 *
 * <p>Basically the point of all the completed code is to allow for changes in the speed of the
 * mechanism to be fed though a filter, a slew rate limiter in this case. The reason why this is
 * tricky is I want the elevator to instantly stop upon hitting a limit switch, however with a
 * filter, this might not always be the case.
 *
 * <p>The code below works as following:
 *
 * <p>When telling the elevator to go up and down we first check to see if the limit switch which
 * regulates this movement is active. If it is, we don't do anything.
 *
 * <p>If it is not active however we set a variable holding our wanted speed. This wanted speed is
 * fed though a filter periodicly to produce the value to set the motors to.
 *
 * <p>If a limit switch becomes active, we check if the applied speed implys that we are moving that
 * direction; if so we insta stop the motors
 *
 * <p>Limit switches can be ignored by setting the "ignore ___ limit" values on shuffleboard
 */
public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {

    StatusManager statusManager = StatusManager.getInstance();

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */

    /* Master Talon */
    private final WPI_TalonFX master; // This is set to 8
    /* Slave Talon */
    private final WPI_TalonFX slave; // This is set to 7

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per
    // second
    private final SlewRateLimiter filter = new SlewRateLimiter(1.75);

    private final NetworkTableEntry ignoreTopLimit;
    private final NetworkTableEntry ignoreBottomLimit;

    // Both of these default to 0
    double wantedSpeed; // The speed / direction we want
    double appliedSpeed; // The current speed being applied to the motors

    // Should only be needed to be called directly during unit tests, else just use the no args
    // constructor
    public ElevatorSubsystem(
            WPI_TalonFX master,
            WPI_TalonFX slave,
            DigitalInput topLimitSwitch,
            DigitalInput bottomLimitSwitch) {
        this.master = master;
        this.slave = slave;
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
        /* Please look at the 2021 code and configure the peak/nominal output and stator current limits */
        /* Also maybe configure velocity/PID values */

        // Configure factory defaults
        this.master.configFactoryDefault();
        this.slave.configFactoryDefault();
        statusManager.logCTREError(this.master);
        statusManager.logCTREError(this.slave);

        // Set to break
        this.master.setNeutralMode(NeutralMode.Brake);
        this.slave.setNeutralMode(NeutralMode.Brake);
        statusManager.logCTREError(this.master);
        statusManager.logCTREError(this.slave);

        // Make slave follow master
        this.slave.follow(this.master);

        // One of the motors needs to be inverted, so we do this
        // if Master is ID 8, use counterclockwise
        // If Master is 7 use Clockwise
        this.slave.setInverted(
                TalonFXInvertType.Clockwise); // If they still move the same way, try clockwise

        ShuffleboardLayout layout =
                Shuffleboard.getTab(TelemetryConstants.SUBSYSTEM_TAB)
                        .getLayout("Elevator", BuiltInLayouts.kGrid);
        layout.addNumber("Speed", this.master::get);
        ignoreTopLimit =
                layout.add("Ignore Top Limit", false)
                        .withWidget(BuiltInWidgets.kToggleButton)
                        .getEntry();
        ignoreBottomLimit =
                layout.add("Ignore Bottom Limit", false)
                        .withWidget(BuiltInWidgets.kToggleButton)
                        .getEntry();

        statusManager.addMotor(this.master, "ELV_M");
        statusManager.addMotor(this.slave, "ELV_S");
    }

    // No args constructor for standard initiation of the class
    public ElevatorSubsystem() {
        this( // Call our other constructor with these hardware inputs
                new WPI_TalonFX(ElevatorConstants.MASTER_PORT), // Master
                new WPI_TalonFX(ElevatorConstants.SLAVE_PORT), // Slave
                new DigitalInput(ElevatorConstants.TOP_LIMIT_PORT), // Top
                new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_PORT) // Bottom
                );
    }
    /** Starts the elevator moving up with the speed specified in constants */
    public void upElevator() {
        if (!atTop()) { // If the top limit switch is not true
            wantedSpeed = ElevatorConstants.UP_SPEED;
            System.out.println("Elevator up");
        }
        ;
    }
    /** Starts the elevator moving down with the speed specified in constants */
    public void downElevator() {
        if (!atBottom()) { // If the bottom limit switch is not true
            wantedSpeed = ElevatorConstants.DOWN_SPEED;
            System.out.println("Elevator down");
        }
    }
    /** Stops the elevator with a ramp down */
    public void stopElevator() { // Ramps down the motors
        wantedSpeed = 0;
        System.out.println("Elevator Stop");
    }
    /** Instantly Stops the elevator */
    public void haltElevator() { // Stops the elevator without ramping.
        filter.reset(0);
        wantedSpeed = 0;
    }

    private void checkLimits() {
        if ((atTop() && Math.signum(appliedSpeed) == Math.signum(ElevatorConstants.UP_SPEED))
                || (atBottom()
                        && Math.signum(appliedSpeed)
                                == Math.signum(
                                        ElevatorConstants
                                                .DOWN_SPEED))) { // Checks to see if the limit
            // switch is active
            haltElevator(); // Stop the elevator
        }
    }

    public void updateSpeed() { // Set the motors to the wanted direction.
        // Math.signum returns -1.0 if the number is negative, 1.0 if positive and 0.0 if 0
        SmartDashboard.putNumber("Wanted", wantedSpeed);
        if (Math.signum(wantedSpeed) == Math.signum(ElevatorConstants.UP_SPEED)
                && !atTop()) { // If we want to move upwards, and we aren't at the top
            appliedSpeed =
                    filter.calculate(wantedSpeed); // Calculate the speed that should be applied
        } else if (Math.signum(wantedSpeed) == Math.signum(ElevatorConstants.DOWN_SPEED)
                && !atBottom()) { // If we want to move downwards, and we aren't at the bottom
            appliedSpeed =
                    filter.calculate(wantedSpeed); // Calculate the speed that should be applied
        } else {
            appliedSpeed = filter.calculate(0); // Calculate the speed that should be applied
        }
        SmartDashboard.putNumber("Applied", appliedSpeed);
        master.set(appliedSpeed);
        statusManager.logCTREError(master);
    }

    public boolean atTop() {
        return !topLimitSwitch.get() && !ignoreTopLimit.getBoolean(false);
    }

    public boolean atBottom() {
        return !bottomLimitSwitch.get() && !ignoreBottomLimit.getBoolean(false);
    }

    @Override
    public void periodic() {
        checkLimits();
        updateSpeed();
    }

    // Close all hardware. needed for testing
    @Override
    public void close() {
        master.close();
        slave.close();
        topLimitSwitch.close();
        bottomLimitSwitch.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }
}
