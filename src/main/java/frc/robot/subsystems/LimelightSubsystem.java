package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTableEntry m_tve, m_txe, m_tye, m_tae;

    @Override
    public void periodic()
    {
        //read values periodically
        double tv   = m_tve.getDouble(0.0);
        double tx   = m_txe.getDouble(0.0);
        double ty   = m_tye.getDouble(0.0);
        double area = m_tae.getDouble(0.0);
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightValid", tv);
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public LimelightSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
        m_tve = table.getEntry("tv");
        m_txe = table.getEntry("tx");
        m_tye = table.getEntry("ty");
        m_tae = table.getEntry("ta");
    }
  }