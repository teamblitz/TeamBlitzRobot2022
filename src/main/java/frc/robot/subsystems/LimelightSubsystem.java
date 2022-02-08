// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class LimelightSubsystem {
//     double controllerScaleFactor = 0.6;
//     double rotationScaleFactor = 0.4;

//     float Kp = -0.1f;
//     float min_command = 0.05f;

//     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//     NetworkTableEntry txe = table.getEntry("tx");
//     NetworkTableEntry tye = table.getEntry("ty");
//     NetworkTableEntry tae = table.getEntry("ta");

//     //read values periodically
//     double tx = txe.getDouble(0.0);
//     double ty = tye.getDouble(0.0);
//     double area = tae.getDouble(0.0);


//     //post to smart dashboard periodically
//     SmartDashboard.putNumber("LimelightX", tx);
//     SmartDashboard.putNumber("LimelightY", ty);
//     SmartDashboard.putNumber("LimelightArea", area);

//     if (m_driveController.getLeftBumper()) 
//     {
//             double heading_error = -tx;
//             double steering_adjust = 0.0f;
//             if (tx > 1.0)
//             {
//                     steering_adjust = Kp*heading_error - min_command;
//             }
//             else if (tx < 1.0)
//             {
//                     steering_adjust = Kp*heading_error + min_command;
//             }
//             m_drive.arcadeDrive(rotationScaleFactor * steering_adjust, controllerScaleFactor * m_driveController.getLeftY());
//     }
//     else
//     {
//         // gets left value and right value from controller (possibly negates one) and passes to tank Drive code
//         // multiply getLeftY and getRightY by a fractional factor like 0.25 to crank down the power/responsiveness for safety
//         m_myRobot.arcadeDrive(rotationScaleFactor * -m_driveController.getLeftX(), controllerScaleFactor * m_driveController.getLeftY());

//     }
// }
