package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;



public class ElevatorSubsystem extends SubsystemBase {
    DigitalInput toplimitSwitch= new DigitalInput(8);
    DigitalInput bottomlimitSwitch = new DigitalInput(7);
    boolean checkMovement = false;
    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */
    
    /* Master Talon */
    private final WPI_TalonFX m_master = new WPI_TalonFX(Constants.ElevatorConstants.kMasterPort); // This is set to 8

    /* Slave Talon */
    private final WPI_TalonFX m_slave = new WPI_TalonFX(Constants.ElevatorConstants.kSlavePort); // This is set to 7
    

    // TODO - <<<>>> Did not create limiters for stoping of the elevator. As the moters would keep going a bit as they slowed down, Posiblly resaulting in the moter going too far. 
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second
    SlewRateLimiter upFilter = new SlewRateLimiter(1.75);
    SlewRateLimiter downFilter = new SlewRateLimiter(1.75);

    private enum  Direction{ // Has 3 states
        UP, // Moving up
        DOWN, // Moving down
        NONE // Not moving
    }

    private Direction direction = Direction.NONE;


    public ElevatorSubsystem() {
        
        /* Please look at the 2021 code and configure the peak/nominal output and stator current limits */
        /* Also maybe configure velocity/PID values */

        // Configure factory defaults
        m_master.configFactoryDefault();
        m_slave.configFactoryDefault();

        // Set to coast
        m_master.setNeutralMode(NeutralMode.Brake); // This might be changed to brake
        m_slave.setNeutralMode(NeutralMode.Brake); // Same with this one

        // Make slave follow master
        m_slave.follow(m_master);
        
        // One of the motors needs to be inverted, so we do this
        // if Master is ID 8, use counterclockwise
        // If Master is 7 use Clockwise
        m_slave.setInverted(TalonFXInvertType.Clockwise); // If they still move the same way, try clockwise


    }

    public void upElevator() {
        if (Robot.isSimulation()) {
            System.out.println("upElevator Called");
            }
        // Drives the motors up (or at least it should)
        // m_master.set(ControlMode.PercentOutput, upFilter.calculate(-0.6));
        
        if (!toplimitSwitch.get()) {// If the top limit switch is not true
            m_master.set(ControlMode.PercentOutput, -0.40); // Turn the moter on.
            direction = Direction.UP; // We are now moving up
            System.out.println("Elevator up");
        }
        checkMovement = true;
    }

    public void downElevator() {
        if (Robot.isSimulation()) {
            System.out.println("downElevator Called");
            }
        // Drives the motors down (or at least it should)
        // m_master.set(ControlMode.PercentOutput, downFilter.calculate(0.6));
        if (!bottomlimitSwitch.get()) { // If the bottem limit switch is not true
            m_master.set(ControlMode.PercentOutput, 0.40); // Turn the moter on.
            direction = Direction.DOWN; // We are now moving dow
            System.out.println("Elevator down");
        }
        // checkMovement = true;
    }

    public void stopElevator() {
        // Should stop the motors
        m_master.set(ControlMode.PercentOutput, 0.0);
        direction = Direction.NONE; // We are no longer moving
        System.out.println("Elevator Stop");
        // checkMovement = false;
    }

    public void checkTopLimit(){
        if (toplimitSwitch.get() && direction == Direction.UP) { // If we are at the top and moving up
            stopElevator(); // Stop the elevator
        }
        // if (toplimitSwitch.get() && checkMovement) { 
        //     stopElevator();
        //     System.out.println("Stop Elevator Up");
        // }
    }

    public void checkBottomLimit(){
        if (bottomlimitSwitch.get() && direction == Direction.DOWN) { // If we are at the bottom and moving down
            stopElevator(); // Stop the elevator
        }
        // if (bottomlimitSwitch.get() && checkMovement) { 
        //     stopElevator();
        //     System.out.println("Stop Elevator Down");
        // }
    }

    @Override
    public void periodic() {
        checkTopLimit();
        checkBottomLimit();
    }
}