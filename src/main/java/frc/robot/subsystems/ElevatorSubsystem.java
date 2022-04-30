package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants;;



public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable{
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");
        
        builder.addStringProperty("Direction", () -> direction.toString(), null);
        builder.addBooleanProperty("Ignore top limit", () -> ignoreTopLimit, null);
        builder.addBooleanProperty("Ignore bottom limit", () -> ignoreBottomLimit, null);
        super.initSendable(builder);
    }

    private final DigitalInput m_toplimitSwitch;
    private final DigitalInput m_bottomlimitSwitch;
    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */
    
    /* Master Talon */
    private final WPI_TalonFX m_master; // This is set to 8
    /* Slave Talon */
    private final WPI_TalonFX m_slave; // This is set to 7
    

     
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second
    SlewRateLimiter filter = new SlewRateLimiter(1.75);

    private final double kUpSpeed = -0.40;
    private final double kDownSpeed = 0.40;

    private boolean ignoreTopLimit = false;
    private boolean ignoreBottomLimit = false;
    private void setIgnoreTopLimit(boolean value) {ignoreTopLimit = value;}
    private void setIgnoreBottomLimit(boolean value) {ignoreBottomLimit = value;}

    private enum  Direction{ // Has 3 states
        UP, // Moving up
        DOWN, // Moving down
        NONE, // Not moving, will ramp down
        STOP // Stoped because at limit switch.
    }
    private Direction direction = Direction.NONE; // We arn't moving yet

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

    public void upElevator() {
        System.out.println("upElevator Called");

        if (!m_toplimitSwitch.get()) {// If the top limit switch is not true
            direction = Direction.UP; // Tell the elevator to move up.
            System.out.println("Elevator up");
        }
    }
    public void downElevator() {
        System.out.println("downElevator Called");

        if (!m_bottomlimitSwitch.get() || ignoreTopLimit) { // If the bottem limit switch is not true
            direction = Direction.DOWN; // Tell the elevator to move down.
            System.out.println("Elevator down");
        }
    }

    public void stopElevator() { // Ramps down the moters
        direction = Direction.NONE; // Tell the elevator to ramp down the moters
        System.out.println("Elevator Stop");
    }

    public void haultElevator() { // Stops the elevator without ramping.
        direction = Direction.STOP; // Tell the elevator to stop without ramping
        // System.out.println("Elevator Stop");
    }

    private void checkLimits() {
        if (m_toplimitSwitch.get() && direction == Direction.UP) { // If we are at the top and moving UP
            haultElevator(); // Stop the elevator
        }
        if (m_bottomlimitSwitch.get() && direction == Direction.DOWN) { // If we are at the bottom moving DOWN
            haultElevator(); // Stop the elevator
        }
        if (m_toplimitSwitch.get() || m_bottomlimitSwitch.get() && direction == Direction.NONE) { // If we are ramping down and touching either limit switch
            // haultElevator(); // Stop the elevator
        }
    }

    public void updateSpeed() { // Set the moters to the wanted direction.
        switch (direction) {
            case UP:
                if (!m_toplimitSwitch.get()) { // If we aren't touching the top
                    m_master.set(ControlMode.PercentOutput, filter.calculate(kUpSpeed)); // Start the elevator up
                }
                break;
            case DOWN:
                if (!m_bottomlimitSwitch.get()) { // If we aren't touching the bottom
                m_master.set(ControlMode.PercentOutput, filter.calculate(kDownSpeed)); // Start the elevator down
                }
                break;
            case NONE:
                m_master.set(ControlMode.PercentOutput, filter.calculate(0.0)); // Set the moters to ramp down
                break;
            case STOP:
                filter.reset(0.0); // reset the slewrate to 0
                m_master.set(ControlMode.PercentOutput, filter.calculate(0.0)); // Stop the moters without ramping
                break;
        }
        
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