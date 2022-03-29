package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Condenses all Limelight and Plan Subsystems into one Subsystem to decrease verbosity 
public class VisionSubsystem extends SubsystemBase{

    private LimelightCamera ballLimeLimelight;
    private LimelightCamera targetLimeLight;


    public VisionSubsystem() {
        ballLimeLimelight = new LimelightCamera("limelight", 0);
    }
    // Instantiated for each limelight
    private class LimelightCamera {

        private NetworkTableEntry m_tve, m_txe, m_tye, m_tae;
        private double m_tv, m_tx, m_ty, m_ta;

        public LimelightCamera(String networkTable, Number pipeline) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(networkTable);
            setPipeline(pipeline);

            m_tv = m_tx = m_ty = m_ta = 0.0;
        
            m_tve = table.getEntry("tv");
            m_txe = table.getEntry("tx");
            m_tye = table.getEntry("ty");
            m_tae = table.getEntry("ta");
        }


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
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNum);

        }


        public void periodic() { // We must call this manualy
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

    }
    
    public class BallAcquirePlan {
        // all of the below can be tinkered with for tuning
        private double m_autoRotationScaleFactor = 0.3;
        private double m_autoMoveScaleFactor = 0.9;
        private double m_Kp = -0.1f;
        private double m_min_command = 0.05f;
        private double m_maxHeadingError = 10.0;
        private double m_maxAreaFraction = 25.0;
        private double m_maxDriveSpeedFraction = 0.45; // how fast we allow the autodrive code to dictate we want to go

        private LimelightCamera m_limelight;

        // these are the calculated movement directives for autodrive
        private double m_fwd = 0;
        private double m_rot = 0;

        public BallAcquirePlan(LimelightCamera limelight) {
            m_limelight = limelight;
        }

        public double getFwd() {return(m_fwd);}
        public double getRot() {return(m_rot);}


        public void periodic() { // We must configure this ourselfs.
            // inform LimeLight of our alliance color
            // configure alliance color (0=blue, 1=red)
            // this refers to files like RoundBlue and RoundRed in the limelight_configuration repository
            m_limelight.setPipeline(m_limelight.getAllianceColor() == 1 ? LimelightSubsystem.kSeekRedContour : LimelightSubsystem.kSeekBlueContour);

            m_rot = 0.0;
            m_fwd = 0.0;
            // acquire tx and area and valid from LimelightSubsystem
            double tx = m_limelight.getX();
            double area = m_limelight.getArea();

            // boolean valid = false;

            if(m_limelight.getValid() > 0)
            {
                double heading_error = -tx; // there's a negative here that's negated again later in arcadeDrive()

                // clamp tx magnitude for safety
                heading_error = Math.max(-m_maxHeadingError, heading_error);
                heading_error = Math.min(heading_error, m_maxHeadingError);
        
                double steering_adjust = 0.0f;
                if (tx > 1.0)
                {
                        steering_adjust = m_Kp*heading_error - m_min_command;
                }
                else if (tx < 1.0)
                {
                        steering_adjust = m_Kp*heading_error + m_min_command;
                }
        
                // calculate a drive speed fractionbased on area -- smaller area = further away = go faster
                double driveSpeedFraction = (m_maxAreaFraction - area) / m_maxAreaFraction; // results in [0.0 ... 1.0]
                // limit the drive speed fraction to m_maxDriveSpeedFraction for safety
                driveSpeedFraction = Math.min(m_maxDriveSpeedFraction, driveSpeedFraction);
        
                // negate steering adjust because camera and ball feeder face the rear
                m_rot = m_autoRotationScaleFactor * -steering_adjust;
                m_fwd = m_autoMoveScaleFactor * driveSpeedFraction;
                
                // we could post the debug info to the Shuffleboard if we wanted
                SmartDashboard.putNumber("AutoMove", (m_autoRotationScaleFactor * driveSpeedFraction));
                
                
            }

        }

        
    }
}
