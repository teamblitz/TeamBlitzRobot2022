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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.FeederArmSubsystem;
// import frc.robot.subsystems.FeederWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.UpperPulleySubsystem;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  //Toggle FeederArm


  // Chasis drive subsystem:
  private DriveSubsystem m_robotDrive;

  // Feeder & shooter subsystm:
  private ShooterSubsystem m_shooter;
  // private FeederArmSubsystem m_intakeArm;
  // private FeederWheelsSubsystem m_intakeRoller;

  //Upper Pulley Subsystem:
  // private UpperPulleySubsystem m_upperPulley;

  // Controllers:
  private XboxController m_driveController;
  private Joystick m_auxiliaryController;

  // Controller Constants:
  private final boolean kUseTankDrive = false;
  // Enables TankDrive
  private final double kLowSpeed = 0.5;
  // When trigger not held this is maximum speed
  private final double kFullSpeed = 1.0;
  // When trigger is held this is maximum speed
  /**
  * The container for the robot. Contains subsystems, OI devices, and commands.
  */

  public RobotContainer() {

    // m_intakeArm = new FeederArmSubsystem();
    // m_feederArmCommand = new ToggleFeederArmCommand(m_intakeArm);


    configureSubsystems();
    configureButtonBindings();
    // Tank drive
    if (kUseTankDrive) {
      /* m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      .tankDrive(m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed),
      //Get y value of left analog stick. Then set the motor speed to a max of 50% when the left trigger is less then half pulled otherwise set the max speed to 100%
      m_driveController.getRightY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
      m_robotDrive));
      //Get y value of right analog stick. Then set the mmotor speed to a max of 50% when the left trigger is less than half pulled otherwise set the max speed to 100%
      */

      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      .tankDrive((m_driveController.getRawAxis(1)),
      //Get y value of left analog stick. Then set the motor speed to a max of 50% when the left trigger is less then half pulled otherwise set the max speed to 100%
      m_driveController.getRawAxis(5)),
      m_robotDrive));
      // .tankDrive(m_driveController.getRawAxis(1), m_driveController.getRawAxis(2));

    }

    else { //arcadeDrive
      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      .arcadeDrive(-m_driveController.getLeftY(),
      -m_driveController.getRightX()),
      m_robotDrive));
    }
  }

  private void configureSubsystems() {

    m_robotDrive = new DriveSubsystem();

    m_shooter = new ShooterSubsystem();

    // m_intakeArm = new FeederArmSubsystem();
    // m_intakeRoller = new FeederWheelsSubsystem();

    // m_upperPulley = new UpperPulleySubsystem();

    m_driveController = new XboxController(OIConstants.kDriveControllerPort);
    m_auxiliaryController = new Joystick(OIConstants.kAncillaryControlerPort);
  }


  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {

      // // ***** FEEDER ARM SUBSYSTEM *****
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederArmDownButton)
      // .whenPressed(new InstantCommand(m_intakeArm::downFeeder, m_intakeArm).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederArmDownButton + " Pressed")));
      // // When button (11) on the joystick is held, the feeder arm will be lowered. Before lowering it will say "Joystick Button (11) Pressed"
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederArmDownButton)
      // .whenReleased(new InstantCommand(m_intakeArm::stopFeeder, m_intakeArm).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederArmDownButton + " Released")));
      // // When button (11) on the joystick is released, the feeder arm will stop lowering. Before stopping it will say "Joystick Button (11) Released"
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederArmUpButton)
      // .whenPressed(new InstantCommand(m_intakeArm::upFeeder, m_intakeArm).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederArmUpButton + " Pressed")));
      // // When button (10) on the joystick is held, the feeder arm will be raised. Before raising it will say "Joystick Button (10) Pressed"
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederArmUpButton)
      // .whenReleased(new InstantCommand(m_intakeArm::stopFeeder, m_intakeArm).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederArmUpButton + " Released")));
      // // When button (10) on the joystick is released, the feeder arm will stop raising. Before stopping it will say "Joystick Button (10) Released"

      // Alternate approach to use a toggle for raise vs lower. For this we would need to set
      // soft limits or use switches to stop the motor
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederArmToggleButton)
      //   .whenPressed(m_feederArmCommand::toggle);

      // ***** FEEDER WHEELS SUBSYSTEM *****
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederIntakeToggleButton)
      // .whenPressed(new InstantCommand(m_intakeRoller::runFeederWheels, m_intakeRoller).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederIntakeToggleButton + " Pressed")));
      // // When Button 3 is held then it will toggle on the ball feeder. Before it will output "Joystick Button (3) Pressed"
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederIntakeToggleOff)
      // .whenPressed(new InstantCommand(m_intakeRoller::stopFeederWheels, m_intakeRoller).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederIntakeToggleButton + " Released")));
      // // When Button 3 is released, then it will toggle off the ball feeder. Before it will output "Joystick Button (3) Released"
      // new JoystickButton(m_auxiliaryController, OIConstants.kFeederIntakeToggleBack)
      // .whenPressed(new InstantCommand(m_intakeRoller::runFeederWheelsback, m_intakeRoller).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kFeederIntakeToggleButton + " Pressed")));
      // // ***** SHOOTER SUBSYSTEM *****
      new JoystickButton(m_auxiliaryController, OIConstants.kShooterToggleButton)
      .whenPressed(new InstantCommand(m_shooter::shoot, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooterToggleButton + " Pressed")));
      // When the trigger is pulled (button 1), the shooter will toggle on. Before it will output "Joystick Button (1) Pressed"
      new JoystickButton(m_auxiliaryController, OIConstants.kShooterToggleOff)
      .whenReleased(new InstantCommand(m_shooter::stopshooter, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooterToggleButton + "  Released")));
      // When the trigger is released (button 1) the shooter will toggle off. Before stopping it will output "Joystick Button (1) Released"

      // ***** UPPER PULLEY SUBSYSTEM *****
      // new JoystickButton(m_auxiliaryController, OIConstants.kUpperPulleyButtonUp)
      // .whenPressed(new InstantCommand(m_upperPulley::upPulley, m_upperPulley).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kUpperPulleyButtonUp + "Pressed")));
      // //When button 12 (I think it's a button on the left side) is pushed, the pulley will move up
      // new JoystickButton(m_auxiliaryController, OIConstants.kUpperPulleyButtonUp)
      // .whenReleased(new InstantCommand(m_upperPulley::stopPulley, m_upperPulley).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kUpperPulleyButtonUp + "Released")));
      // //When button 12 is released, the pulley will stop moving
      // new JoystickButton(m_auxiliaryController, OIConstants.kUpperPulleyButtonDown)
      // .whenPressed(new InstantCommand(m_upperPulley::downPulley, m_upperPulley).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kUpperPulleyButtonDown + "Pressed")));
      // new JoystickButton(m_auxiliaryController, OIConstants.kUpperPulleyButtonDown)
      // .whenReleased(new InstantCommand(m_upperPulley::stopPulley, m_upperPulley).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kUpperPulleyButtonUp + "Released")));
      // //When button 13 (Another button on the left, maybe) is pushed, the pulley will move down
      // new JoystickButton(m_auxiliaryController, OIConstants.kUpperPulleyButtonDown)
      // .whenReleased(new InstantCommand(m_upperPulley::stopPulley, m_upperPulley).beforeStarting (() -> System.out.println("Joystick Button " + OIConstants.kUpperPulleyButtonDown + "Released")));
      //When button 13 is released, the pulley will stop moving

     //Zone Controls:
     new JoystickButton(m_driveController, OIConstants.kRedZone)
      .whenPressed(new InstantCommand(m_shooter::redZone, m_shooter).beforeStarting(() -> System.out.println("Xbox Button " + OIConstants.kRedZone + "Pressed"))); 
    new JoystickButton(m_driveController, OIConstants.kBlueZone)
      .whenPressed(new InstantCommand(m_shooter::blueZone, m_shooter).beforeStarting(() -> System.out.println("Xbox Button " + OIConstants.kBlueZone + "Pressed"))); 
    new JoystickButton(m_driveController, OIConstants.kYellowZone)
      .whenPressed(new InstantCommand(m_shooter::yellowZone, m_shooter).beforeStarting(() -> System.out.println("Xbox Button " + OIConstants.kYellowZone + "Pressed"))); 
    new JoystickButton(m_driveController, OIConstants.kGreenZone)
      .whenPressed(new InstantCommand(m_shooter::greenZone, m_shooter).beforeStarting(() -> System.out.println("Xbox Button " + OIConstants.kGreenZone + "Pressed"))); 
    new JoystickButton(m_driveController, OIConstants.kShuffleboardZone)
      .whenPressed(new InstantCommand(m_shooter::shuffleboardZone, m_shooter).beforeStarting(() -> System.out.println("Xbox Button " + OIConstants.kShuffleboardZone + "Pressed"))); 
    
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

