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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TelementryConstants;
import frc.robot.Constants.ElevatorConstants;


/**
 * Welcome to this mess I have been working on.
 * 
 * Basicly the point of all the complectated code is to allow for changes in the speed of 
 * the mechanism to be fed though a filter, a slew rate limiter in this case.
 * The reason why this is tricky is I want the elevator to instantly stop upon hitting a limit switch,
 * however with a filter, this might not always be the case.
 * 
 * The code below works as following:
 * 
 * When telling the elevator to go up and down we first check to see if the limit switch which regulates this movement is active.
 * If it is, we don't do anything.
 * 
 * If it is not active however we set a variable holding our wanted speed. This wanted speed is fed though a filter periodicly
 * to produce the value to set the motors to.
 * 
 * If a limit switch becomes active, we check if the applied speed implys that we are moving that direction; if so we insta stop the motors
 * 
 * Limit switches can be ignored by setting the "ignore ___ limit" values on shuffleboard
 */
public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
    
    private final DigitalInput m_toplimitSwitch;
    private final DigitalInput m_bottomlimitSwitch;
    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */
    
    /* Master Talon */
    private final WPI_TalonFX m_master; // This is set to 8
    /* Slave Talon */
    private final WPI_TalonFX m_slave; // This is set to 7
    

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second
    private final SlewRateLimiter filter = new SlewRateLimiter(1.75);


    private NetworkTableEntry ignoreTopLimit;
    private NetworkTableEntry ignoreBottomLimit;


    // Both of these default to 0
    double wantedSpeed; // The speed / direction we want
    double applliedSpeed; // The current speed being applyed to the motors


    // Should only be needed to be called directly durring unit tests, else just use the no args constructor
    public ElevatorSubsystem(WPI_TalonFX master, WPI_TalonFX slave, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
        m_master = master;
        m_slave = slave;
        m_toplimitSwitch = topLimitSwitch;
        m_bottomlimitSwitch = bottomLimitSwitch;
        /* Please look at the 2021 code and configure the peak/nominal output and stator current limits */
        /* Also maybe configure velocity/PID values */

        // Configure factory defaults
        m_master.configFactoryDefault();
        m_slave.configFactoryDefault();

        // Set to break
        m_master.setNeutralMode(NeutralMode.Brake); // This might be changed to brake
        m_slave.setNeutralMode(NeutralMode.Brake); // Same with this one

        // Make slave follow master
        m_slave.follow(m_master);
        
        // One of the motors needs to be inverted, so we do this
        // if Master is ID 8, use counterclockwise
        // If Master is 7 use Clockwise
        m_slave.setInverted(TalonFXInvertType.Clockwise); // If they still move the same way, try clockwise  

        ShuffleboardLayout layout = Shuffleboard.getTab(TelementryConstants.kSubsystemTab).getLayout("Elevator", BuiltInLayouts.kGrid);
        layout.addNumber("Speed", m_master::get);
        ignoreTopLimit = layout.add("Ignore Top Limit", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        ignoreBottomLimit = layout.add("Ignore Bottom Limit", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    // No args constructor for standerd initation of the class
    public ElevatorSubsystem() {
        this( // Call our other with these hardwere inputs
            new WPI_TalonFX(ElevatorConstants.kMasterPort), // Master
            new WPI_TalonFX(ElevatorConstants.kSlavePort), // Slave
            new DigitalInput(ElevatorConstants.kTopLimitPort), // Top
            new DigitalInput(ElevatorConstants.kBottomLimitPort) // Bottom
        );
    }
    /**
     * Starts the elevator moving up with the speed specified in constants
     */
    public void upElevator() {
        if (!atTop()) {// If the top limit switch is not true
            wantedSpeed = ElevatorConstants.kUpSpeed;
            System.out.println("Elevator up");
        };
    }
    /**
     * Starts the elevator moving down with the speed specified in constants
     */
    public void downElevator() {
        if (!atBottom()) { // If the bottem limit switch is not true
            wantedSpeed = ElevatorConstants.kDownSpeed;
            System.out.println("Elevator down");
        }
    }
    /**
     * Stops the elevator with a ramp down
     */
    public void stopElevator() { // Ramps down the moters
        wantedSpeed = 0;
        System.out.println("Elevator Stop");
    }
    /**
     * Instantly Stops the elevator
     */
    public void haultElevator() { // Stops the elevator without ramping.
        filter.reset(0);
        wantedSpeed = 0;
    }

    private void checkLimits() {
        if ((atTop() 
            && Math.signum(applliedSpeed) == Math.signum(ElevatorConstants.kUpSpeed))
            || (atBottom() 
            && Math.signum(applliedSpeed) == Math.signum(ElevatorConstants.kDownSpeed))
            ) { // Checks to see if the limit switch is active 
            haultElevator(); // Stop the elevator
        }
    }

    public void updateSpeed() { // Set the moters to the wanted direction.
        // Math.signum returns -1.0 if the number is negigtive, 1.0 if positive and 0.0 if 0

        if (Math.signum(wantedSpeed) == Math.signum(ElevatorConstants.kUpSpeed) && !atTop()) { // If we want to move upwards and we arn't at the top
            applliedSpeed = filter.calculate(wantedSpeed); // Calculate the speed that should be applied
        } else if (Math.signum(wantedSpeed) == Math.signum(ElevatorConstants.kDownSpeed) && !atBottom()) { // If we want to move downwards and we arn't at the bottom
            applliedSpeed = filter.calculate(wantedSpeed); // Calculate the speed that should be applied
        } else {
            applliedSpeed = filter.calculate(0); // Calculate the speed that should be applied
        }
        m_master.set(applliedSpeed);
        
    }

    private boolean atTop() {
        return m_toplimitSwitch.get() && !ignoreTopLimit.getBoolean(false);
    }
    
    private boolean atBottom() {
        return m_toplimitSwitch.get() && !ignoreBottomLimit.getBoolean(false);
    }

    @Override
    public void periodic() {
        checkLimits();
        updateSpeed();
    }
    
    // Close all hardwere. needed for testing
    @Override
    public void close() {
        m_master.close();
        m_slave.close();
        m_toplimitSwitch.close();
        m_bottomlimitSwitch.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }
}