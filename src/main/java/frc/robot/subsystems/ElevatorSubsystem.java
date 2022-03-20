package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ElevatorSubsystem extends SubsystemBase {
    DigitalInput toplimitSwitch= new DigitalInput(8);
    DigitalInput bottomlimitSwitch = new DigitalInput(7);
    boolean checkMovement = false;
    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */
    
    /* Master Talon */
    private final WPI_TalonFX m_master = new WPI_TalonFX(Constants.ElevatorConstants.kMasterPort); // This is set to 7

    /* Slave Talon */
    private final WPI_TalonFX m_slave = new WPI_TalonFX(Constants.ElevatorConstants.kSlavePort); // This is set to 8
    

    // TODO - <<<>>> Did not create limiters for stoping of the elevator. As the moters would keep going a bit as they slowed down, Posiblly resaulting in the moter going too far. 
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second
    SlewRateLimiter upFilter = new SlewRateLimiter(1.75);
    SlewRateLimiter downFilter = new SlewRateLimiter(1.75);

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

        m_master.setSafetyEnabled(true); // Enable moter saftey so our moters stop if for some reason our code stops.
        m_slave.setSafetyEnabled(true);


    }

    public void upElevator() {
        // Drives the motors up (or at least it should)
        // m_master.set(ControlMode.PercentOutput, upFilter.calculate(-0.6));
        m_master.set(ControlMode.PercentOutput, 0.6); 
        checkMovement = true;
    }

    public void downElevator() {
        // Drives the motors down (or at least it should)
        // m_master.set(ControlMode.PercentOutput, downFilter.calculate(0.6));
        m_master.set(ControlMode.PercentOutput, -0.6);
        checkMovement = true;
    }

    public void stopElevator() {
        // Should stop the motors
        m_master.set(ControlMode.PercentOutput, 0.0);
        checkMovement = false;
    }

    public void checkTopLimit(){
        if (toplimitSwitch.get() && checkMovement) { 
            stopElevator();
            System.out.println("Stop Elevator Up");
        }
    }

    public void checkBottomLimit(){
        if (bottomlimitSwitch.get() && checkMovement) { 
            stopElevator();
            System.out.println("Stop Elevator Down");
        }
    }

    @Override
    public void periodic() {
      checkTopLimit();
      checkBottomLimit();

      m_master.feed(); // Tell our moters we are still alive.
      m_slave.feed();
    }
}