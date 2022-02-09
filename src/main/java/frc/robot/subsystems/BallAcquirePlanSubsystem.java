package frc.robot.subsystems;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallAcquirePlanSubsystem extends SubsystemBase {

    private boolean m_targetLocked = false;

    // all of the below can be tinkered with for tuning
    private double m_autoRotationScaleFactor = 0.3;
    private double m_autoMoveScaleFactor = 0.9;
    private double m_Kp = -0.1f;
    private double m_min_command = 0.05f;
    private double m_maxHeadingError = 10.0;
    private double m_maxAreaFraction = 25.0;
    private double m_maxDriveSpeedFraction = 0.45; // how fast we allow the autodrive code to dictate we want to go

    // these are the calculated movement directives for autodrive
    private double m_fwd = 0;
    private double m_rot = 0;

    public double getFwd() {return(m_fwd);}
    public double getRot() {return(m_rot);}

    @Override
    public void periodic() {
        // TODO <<<>>> acquire tx and area
        double tx = 0.0;
        double area = 0.0;

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
        //SmartDashboard.putNumber("AutoMove", (m_autoRotationScaleFactor * driveSpeedFraction));
    }

    public BallAcquirePlanSubsystem() {

    }
    
}
