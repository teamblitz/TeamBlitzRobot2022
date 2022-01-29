/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

//import org.usfirst.frc.team2083.autocommands.DriveStraightWithDelay;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  // Chasis drive subsystem:
  private DriveSubsystem m_robotDrive;

  // Controllers:
  private XboxController m_driveController;

  // Controller Constants:
  private final boolean kUseTankDrive = false;

  private final double kLowSpeed = 0.5;
  private final double kFullSpeed = 1.0;

  public RobotContainer() {

    configureSubsystems();
    configureButtonBindings();
    /** SlewRateLimiter
    * Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second for forward and backword
    * Essemtaly stoping jerking of the robot durring arcade drive
    * Do Not deleate unless removed below
    */
    // Drive SlewRateLimiter
    SlewRateLimiter filter = new SlewRateLimiter(1.75);
    // Turn SlewRateLimiter
    SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);
    // Tank drive
    if (kUseTankDrive) {
      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      .tankDrive(m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed),
      //Get y value of left analog stick. Then set the motor speed to a max of 50% when the left trigger is less then half pulled otherwise set the max speed to 100%
      m_driveController.getRightY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
      m_robotDrive));
      //Get y value of right analog stick. Then set the mmotor speed to a max of 50% when the left trigger is less than half pulled otherwise set the max speed to 100%
      // m_robotDrive.setDefaultCommand(
      // new RunCommand(() -> m_robotDrive
      // .tankDrive((m_driveController.getRawAxis(1)),
      // //Get y value of left analog stick. Then set the motor speed to a max of 50% when the left trigger is less then half pulled otherwise set the max speed to 100%
      // m_driveController.getRawAxis(5)),
      // m_robotDrive));
      // // .tankDrive(m_driveController.getRawAxis(1), m_driveController.getRawAxis(2));

    }

    else { //arcadeDrive
      // m_robotDrive.setMaxOutput(0.50);
      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
      .arcadeDrive(filter.calculate(-m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
      filterRotation.calculate(-m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed))),
      m_robotDrive));

      // // m_robotDrive.setMaxOutput(0.50);
      // m_robotDrive.setDefaultCommand(
      // new RunCommand(() -> m_robotDrive
      // .arcadeDrive(-m_driveController.getLeftY(),
      // -m_driveController.getRightX()),
      // m_robotDrive));
    }
  }

  private void configureSubsystems() {

    m_robotDrive = new DriveSubsystem();

    m_driveController = new XboxController(OIConstants.kDriveControllerPort);
  
  }


  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      // return new DriveStraightWithDelay(m_robotDrive, 1000, 0.8, 0);  // duration, voltage, delay

      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                                   Constants.DriveConstants.kvVoltsSecondsPerMeter,
                                   Constants.DriveConstants.kaVoltsSecondsSquaredPerMeter),
        Constants.DriveConstants.kDriveKinematics,
        10);
      TrajectoryConfig config = new TrajectoryConfig(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                                                     Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
        // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1),
                new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
        );
      RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                                   Constants.DriveConstants.kvVoltsSecondsPerMeter,
                                   Constants.DriveConstants.kaVoltsSecondsSquaredPerMeter),
                                   Constants.DriveConstants.kDriveKinematics,
                                   m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
      );

      // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
  }

