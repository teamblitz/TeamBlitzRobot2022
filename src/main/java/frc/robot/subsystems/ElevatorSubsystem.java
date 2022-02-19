package frc.robot.subsystems;

import java.util.ResourceBundle.Control; // Unsure if this is necessary

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    /* ***** ----- Talon IDs need to be configured with the Phoenix Tuner ----- ***** */
    
    /* Master Talon */
    private final WPI_TalonFX m_master = new WPI_TalonFX(Constants.ElevatorConstants.kMasterPort); // This is set to 7

    /* Slave Talon */
    private final WPI_TalonFX m_slave = new WPI_TalonFX(Constants.ElevatorConstants.kSlavePort); // This is set to 8

    public ElevatorSubsystem() {
        
        /* Please look at the 2021 code and configure the peak/nominal output and stator current limits */
        /* Also maybe configure velocity/PID values */

        // Configure factory defaults
        m_master.configFactoryDefault();
        m_slave.configFactoryDefault();

        // Set to coast
        m_master.setNeutralMode(NeutralMode.Coast); // This might be changed to brake
        m_slave.setNeutralMode(NeutralMode.Coast); // Same with this one

        // Make slave follow master
        m_slave.follow(m_master);
        
        // One of the motors needs to be inverted, so we do this
        m_slave.setInverted(TalonFXInvertType.CounterClockwise); // If they still move the same way, try clockwise
    }

    public void upElevator() {
        // Drives the motors up (or at least it should)
        m_master.set(ControlMode.PercentOutput, 0.45); 
        
        // Please change the set inverted value for slave/master instead of this percentage
        // Basically, don't do a negative percentage for up and a positive percentage for down
        // You can change the actual value of .45 though.
    }

    public void downElevator() {
        // Drives the motors down (or at least it should)
        m_master.set(ControlMode.PercentOutput, -0.45);
    }

    public void stopElevator() {
        // Should stop the motors
        m_master.set(ControlMode.PercentOutput, 0.0);
    }
}