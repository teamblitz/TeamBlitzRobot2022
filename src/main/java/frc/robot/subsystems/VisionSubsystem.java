package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Condenses all Limelight and Plan Subsystems into one Subsystem to decrease verbosity
 * Should function exactly like the pre existing subsystes. 
 * Outside code using vision must be changed to VisionSubsystem.BallAcquire instead of BallAcquire subsystem. All internal methods and code reamain intact 
 */ 
public class VisionSubsystem extends SubsystemBase{
    // The object that these variables referance shouldn't change therefor we use the final keyword.
    // Final doesn't stop the object from changing internaly, it just stops the referance variable from changing.
    // This way we don't accdently change them externaly.
    public final LimelightCamera ballLimelight;
    public final LimelightCamera targetLimelight;
    public final BallAcquirePlan ballAcquirePlan;
    public final BallShooterPlan ballShooterPlan;

    private final StatusLightSubsystem m_statusLights;
    private final PowerDistribution m_pd;

    public VisionSubsystem(StatusLightSubsystem statusLights, PowerDistribution pd) {

        m_statusLights = statusLights;
        m_pd = pd;

        ballLimelight = new LimelightCamera("limelight", 0); // Idealy these values would come from constants
        targetLimelight = new LimelightCamera("limelight-target", 7);
        
        // We don't need to pass these but keep it this way incase we move these classes.
        ballAcquirePlan = new BallAcquirePlan(ballLimelight, m_statusLights);
        ballShooterPlan = new BallShooterPlan(targetLimelight, m_statusLights);
    }

    @Override
    public void periodic() {
        // Call the nested classes execute method for internal calculation
        ballLimelight.execute();
        targetLimelight.execute();
        ballAcquirePlan.execute();
        ballShooterPlan.execute();
    }

    public void statusLightsOff() {
        m_statusLights.clear();
    }

    public void lightsOn() {
        m_pd.setSwitchableChannel(true);
    }

    public void lightsOff() {
        m_pd.setSwitchableChannel(false);
    }

    // We could move all of the below to its own file without breaking much
    // <<<>>> TODO - Tidy Up Vision Code.
    // Instantiated for each limelight
    public class LimelightCamera {

        private final NetworkTableEntry m_tve, m_txe, m_tye, m_tae;
        private final NetworkTable table; // This must be accesable outside of the constructor
        
        private double m_tv, m_tx, m_ty, m_ta;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private LimelightCamera(String networkTable, Number pipeline) {
            table = NetworkTableInstance.getDefault().getTable(networkTable); 
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
        // TODO - <<<>>> Move this out of this class as it has nothing to do with limelight
        public int getAllianceColor() {
            return(NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false) ? 1 : 0);
        }

        // pipelines selected by below
        public static final int kSeekBlueContour = 0; // Keep these static. Idealy they would be in constants
        public static final int kSeekRedContour = 1;
        public static final int kSeekBlueCircleBlob = 2;
        public static final int kSeekRedCircleBlob = 3;
        // set vision navigation pipeline
        public void setPipeline(Number pipelineNum) {
            // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNum);
            table.getEntry("pipeline").setNumber(pipelineNum);

        }

        public void execute() { // We must call this manualy
            // read values periodically
            // We only need to do this if we are putting to shuffleboard. else we can do this in the getter methods.
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
    
    // Instantiated only once
    public class BallAcquirePlan {
        // all of the below can be tinkered with for tuning
        // Should idealy be in constants.
        private final double m_autoRotationScaleFactor = 0.3;
        private final double m_autoMoveScaleFactor = 0.9;
        private final double m_Kp = -0.1f; // Why are we using the float indicator here? this variable is a double.
        private final double m_min_command = 0.05f;
        private final double m_maxHeadingError = 10.0;
        private final double m_maxAreaFraction = 25.0;
        private final double m_maxDriveSpeedFraction = 0.45; // how fast we allow the autodrive code to dictate we want to go

        private final LimelightCamera m_limelight;
        // We do have this instantiated in the outer class. This will over-ride that instantation for within this class,
        // Keep this here for compatibilty reasons incase we move the class.
        private final StatusLightSubsystem m_statusLights; 

        // these are the calculated movement directives for autodrive
        private double m_fwd = 0;
        private double m_rot = 0;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private BallAcquirePlan(LimelightCamera limelight, StatusLightSubsystem statusLights) {
            m_limelight = limelight;
            m_statusLights = statusLights;
        }

        public double getFwd() {return(m_fwd);}
        public double getRot() {return(m_rot);}

        public void execute() { // We must call this ourselfs.

            // inform LimeLight of our alliance color
            // configure alliance color (0=blue, 1=red)
            // this refers to files like RoundBlue and RoundRed in the limelight_configuration repository
            m_limelight.setPipeline(m_limelight.getAllianceColor() == 1 ? LimelightCamera.kSeekRedContour : LimelightCamera.kSeekBlueContour);

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
                m_rot = m_autoRotationScaleFactor * steering_adjust;
                m_fwd = m_autoMoveScaleFactor * -driveSpeedFraction;
                
                // we could post the debug info to the Shuffleboard if we wanted
                SmartDashboard.putNumber("AutoMove", (m_autoRotationScaleFactor * driveSpeedFraction));
            }
        }

        public void statusLights() {
            m_statusLights.setStatusLights(m_limelight.getX() / 25.0 , m_limelight.getArea() / 30.0, m_limelight.getAllianceColor());
        }
    }

    // Instantiated only once
    public class BallShooterPlan {
        // all of the below can be tinkered with for tuning
        // These should Idealy be in constants.
        private final double m_autoRotationScaleFactor = 0.3;
        private final double m_autoMoveScaleFactor = 0.9;
        private final double m_Kp = -0.1f; // Why are we using the float indicator here? this variable is a double.
        private final double m_min_command = 0.05f;
        private final double m_maxHeadingError = 10.0;
        private final double m_maxOffsetFraction = 2.0;
        private final double m_maxDriveSpeedFraction = 0.45; // how fast we allow the autodrive code to dictate we want to go

        private final LimelightCamera m_limelight;
        // We do have this instantiated in the outer class. This will over-ride that instantation for within this class,
        // Keep this here for compatibilty reasons incase we move the class.
        private final StatusLightSubsystem m_statusLights; 

        // these are the calculated movement directives for autodrive
        private double m_fwd = 0;
        private double m_rot = 0;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private BallShooterPlan(LimelightCamera limelight, StatusLightSubsystem statusLights) {
            m_limelight = limelight;
            m_statusLights = statusLights;
        }

        public double getFwd() {return(m_fwd);}
        public double getRot() {return(m_rot);}
        
        public void execute() { // We must call this manualy
            // TODO <<<>>> set correct pipeline number
            // No need to update this periodicly
            // m_limelight.setPipeline(7);

            m_rot = 0.0;
            m_fwd = 0.0;
            // acquire tx and area and valid from LimelightSubsystem
            double tx = m_limelight.getX();
            double ty = m_limelight.getY();

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

                // calculate the direction we need to drive to aim the target where we want it
                // if target is below crosshair, then ty will be negative, and otherwise positive.
                // if ty is negative then we are too far back and need to move forward, so direction will be positive 1
                // if ty is positive then we are too close and need to move back so direction will be negative 1
                double driveDirection = (ty < -1 ? 1 : -1);
                // calculate the speed to drive at based on how far off from target we are
                double driveSpeedFraction = 0.33 * (Math.abs(ty)) / m_maxOffsetFraction; // results in [0.0 ... 1.0]
                // limit the drive speed fraction to m_maxDriveSpeedFraction for safety
                driveSpeedFraction = Math.min(m_maxDriveSpeedFraction, driveSpeedFraction);
                
                // DON'T negate steering adjust because target camera faces the front
                m_rot = m_autoRotationScaleFactor * steering_adjust * -1;
                // -1 below flips the drive direction compared to BallAcquire which faces the rear instead of the front
                m_fwd = m_autoMoveScaleFactor * driveSpeedFraction * driveDirection * -1;

                // we could post the debug info to the Shuffleboard if we wanted
                SmartDashboard.putNumber("AutoTarget", (m_autoRotationScaleFactor * driveSpeedFraction * -1));
                SmartDashboard.putNumber("GetFwd", (m_fwd));
            }
        }

        public void statusLights() {
            m_statusLights.setStatusLights(m_limelight.getX() / 25.0, 1.0 - Math.abs(m_limelight.getY())/20.0, 2);
        }
    }
}
