package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallShooterPlanSubsystem extends SubsystemBase {

    // all of the below can be tinkered with for tuning
    private double m_autoRotationScaleFactor = 0.3;
    private double m_autoMoveScaleFactor = 0.9;
    private double m_Kp = -0.1f;
    private double m_min_command = 0.05f;
    private double m_maxHeadingError = 10.0;
    private double m_maxOffsetFraction = 2.0;
    private double m_maxDriveSpeedFraction = 0.45; // how fast we allow the autodrive code to dictate we want to go

    private LimelightTargetSubsystem m_LimelightTargetSubsystem;

    // these are the calculated movement directives for autodrive
    private double m_fwd = 0;
    private double m_rot = 0;

    public double getFwd() {return(m_fwd);}
    public double getRot() {return(m_rot);}
    

    @Override
    public void periodic() {
        // TODO <<<>>> set correct pipeline number
        m_LimelightTargetSubsystem.setPipeline(7);

        m_rot = 0.0;
        m_fwd = 0.0;
        // acquire tx and area and valid from LimelightSubsystem
        double tx = m_LimelightTargetSubsystem.getX();
        double ty = m_LimelightTargetSubsystem.getY();

        // boolean valid = false;

        if(m_LimelightTargetSubsystem.getValid() > 0)
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

    public BallShooterPlanSubsystem(LimelightTargetSubsystem lSub) {
        m_LimelightTargetSubsystem = lSub;
    }

}
