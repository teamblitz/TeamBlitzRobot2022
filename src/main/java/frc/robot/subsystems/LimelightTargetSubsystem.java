package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Class for limelight that is used to target the reflective tape
@Deprecated(forRemoval=true)
public class LimelightTargetSubsystem extends SubsystemBase {
  /**
  *    @deprecated Please use {@link VisionSubsystem} and it's inner classes instead.
  *    As it has up to date vision code.
  **/
  private NetworkTableEntry m_tve, m_txe, m_tye, m_tae;
  private double m_tv, m_tx, m_ty, m_ta;

  public double getValid() { return(m_tv);}
  public double getX() { return(m_tx);}
  public double getY() { return(m_ty);}
  public double getArea() { return(m_ta);}


  // returns 0 for blue, 1 for red
  public int getAllianceColor() {
    return(NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false) ? 1 : 0);
  }

  // pipelines selected by below
  public static final int kSeekBlueContour = 0;
  public static final int kSeekRedContour = 1;
  public static final int kSeekBlueCircleBlob = 2;
  public static final int kSeekRedCircleBlob = 3;
  // set vision navigation pipeline
  public void setPipeline(Number pipelineNum) {
    NetworkTableInstance.getDefault().getTable("limelight-target").getEntry("pipeline").setNumber(pipelineNum);

  }

    @Override
    public void periodic() {
        //read values periodically
        m_tv = m_tve.getDouble(0.0);
        m_tx = m_txe.getDouble(0.0);
        m_ty = m_tye.getDouble(0.0);
        m_ta = m_tae.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightTargetValid", m_tv);
        SmartDashboard.putNumber("LimelightTargetX", m_tx);
        SmartDashboard.putNumber("LimelightTargetY", m_ty);
        SmartDashboard.putNumber("LimelightTargetArea", m_ta);
    }

    public LimelightTargetSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-target");

        m_tv = m_tx = m_ty = m_ta = 0.0;

        m_tve = table.getEntry("tv");
        m_txe = table.getEntry("tx");
        m_tye = table.getEntry("ty");
        m_tae = table.getEntry("ta");
    }
  }