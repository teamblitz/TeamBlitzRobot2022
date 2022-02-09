package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTableEntry m_tve, m_txe, m_tye, m_tae;
  private double m_tv, m_tx, m_ty, m_ta;

  public double getValid() { return(m_tv);}
  public double getX() { return(m_tx);}
  public double getY() { return(m_ty);}
  public double getArea() { return(m_ta);}

    @Override
    public void periodic() {
        //read values periodically
        m_tv = m_tve.getDouble(0.0);
        m_tx = m_txe.getDouble(0.0);
        m_ty = m_tye.getDouble(0.0);
        m_ta = m_tae.getDouble(0.0);
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightValid", m_tv);
        SmartDashboard.putNumber("LimelightX", m_tx);
        SmartDashboard.putNumber("LimelightY", m_ty);
        SmartDashboard.putNumber("LimelightArea", m_ta);
    }

    public LimelightSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        m_tv = m_tx = m_ty = m_ta = 0.0;
    
        m_tve = table.getEntry("tv");
        m_txe = table.getEntry("tx");
        m_tye = table.getEntry("ty");
        m_tae = table.getEntry("ta");
    }
  }