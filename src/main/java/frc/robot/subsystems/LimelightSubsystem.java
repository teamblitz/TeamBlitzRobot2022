package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private boolean m_targetLockNotify = false;

    public LimelightSubsystem() {

        double moveScaleFactor = 0.6;
        double rotationScaleFactor = 0.4;
        double autoRotationScaleFactor = 0.3;
        double autoMoveScaleFactor = 0.9;
        
    
        float Kp = -0.1f;
        float min_command = 0.05f;
    
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
        NetworkTableEntry tve = table.getEntry("tv");
        NetworkTableEntry txe = table.getEntry("tx");
        NetworkTableEntry tye = table.getEntry("ty");
        NetworkTableEntry tae = table.getEntry("ta");
    
        //read values periodically
        double tv = tve.getDouble(0.0);
        double tx = txe.getDouble(0.0);
        double ty = tye.getDouble(0.0);
        double area = tae.getDouble(0.0);
    
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightValid", tv);
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", area);
    
        // toggle target lock notification on right bumper
        if(m_driveController.getRightBumperReleased())
        {
          m_targetLockNotify = !m_targetLockNotify;
        }
        
        // notify if target locked
        if (!m_driveController.getLeftBumper() && (tv > 0) && m_targetLockNotify)
        {
          m_driveController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
        }
    
        // execute auto acquire of ball on left bumper
        if (m_driveController.getLeftBumper() && (tv > 0))
        {
                double heading_error = -tx; // there's a negative here that's negated again later in arcadeDrive()
    
                // clamp tx magnitude for safety
                heading_error = Math.max(-10, heading_error);
                heading_error = Math.min(heading_error, 10);
    
                double steering_adjust = 0.0f;
                if (tx > 1.0)
                {
                        steering_adjust = Kp*heading_error - min_command;
                }
                else if (tx < 1.0)
                {
                        steering_adjust = Kp*heading_error + min_command;
                }
    
                // calculate a drive speed fraction
                double driveSpeedFraction = (25.0 - area) / 25.0; // results in [0.0 ... 1.0]
                driveSpeedFraction = Math.min(0.45, driveSpeedFraction);
    
                // negate steering adjust because camera and ball feeder face the rear
                m_myRobot.arcadeDrive(autoRotationScaleFactor * -steering_adjust, autoMoveScaleFactor * driveSpeedFraction);
                SmartDashboard.putNumber("AutoMove", (autoMoveScaleFactor * driveSpeedFraction));
        }
        else
        {
            // gets left value and right value from controller (possibly negates one) and passes to tank Drive code
            // multiply getLeftY and getRightY by a fractional factor like 0.25 to crank down the power/responsiveness for safety
            m_myRobot.arcadeDrive(rotationScaleFactor * -m_driveController.getLeftX(), moveScaleFactor * m_driveController.getLeftY());

        }
    }
  }