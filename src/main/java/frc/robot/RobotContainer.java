/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
// import frc.robot.commands.AutonomousCommand; //test thing
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.commands.SeekBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.Target;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightTargetSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusLightSubsystem;
import frc.robot.subsystems.BallAcquirePlanSubsystem;
import frc.robot.subsystems.BallMoverSubsystem;
import frc.robot.subsystems.BallShooterPlanSubsystem;
import frc.robot.subsystems.InternalBallDetectorSubsystem;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  // Power Board
  private PowerDistribution m_PD;

  // Chassis drive subsystem:
  private DriveSubsystem m_robotDrive;

  // Limelight subsystem:
  private LimelightSubsystem m_limelight;

  // Targeting Limelight subsystem
  private LimelightTargetSubsystem m_limelightTarget;

  // Color sensor
  private InternalBallDetectorSubsystem m_internalBallDetector; 

  // Status Light Leds
  private StatusLightSubsystem m_statusLightSubsystem;

  // Ball Acquire subsystem:
  private BallAcquirePlanSubsystem m_ballAcquire;

  // Ball Shooter Plan Subsystem
  private BallShooterPlanSubsystem m_ballShoot;

  // Elevator Subsystem:
  private ElevatorSubsystem m_elevator;

  // Shooter Subsystem
  private ShooterSubsystem m_shooter;

  // Controllers:
  private XboxController m_driveController;

  // Controller Constants:
  private IntakeSubsystem m_intakeRoller;
  private BallMoverSubsystem m_ballMover;

  private final double kDriveLowSpeed = 0.75;
  private final double kDriveFullSpeed = 1.0;

  private final double kTurnLowSpeed = 0.45;
  private final double kTurnFullSpeed = .60;

  public RobotContainer() {

    configureSubsystems();
    configureButtonBindings();
    CameraServer.startAutomaticCapture();
    m_robotDrive.setDefaultCommand(new RunCommand(() -> m_robotDrive.seed(), m_robotDrive));
    m_PD.setSwitchableChannel(false); // Turn off our light
  }

  // public void beginTeleop(){
  //   System.out.println("Enabling controller for Teleop");

  // /** SlewRateLimiter
  //   * Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second for forward and backward
  //   * Essemtaly stoping jerking of the robot during arcade drive
  //   * Do Not delete unless removed below
  //   */

  //   // Drive SlewRateLimiter
  //   SlewRateLimiter filter = new SlewRateLimiter(1.75);
  //   // Turn SlewRateLimiter
  //   SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);

  //   m_robotDrive.setDefaultCommand(
  //     new RunCommand(() -> m_robotDrive
  //     // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
  //       .performDrive(
  //         filter.calculate(-m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
  //         filterRotation.calculate(m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)), 
  //         m_driveController.getLeftBumper(), //Turns on semiautonomous ball acquire
  //         m_driveController.getLeftTriggerAxis() > 0.5), //Turns on semiautonomous targeter on Left Trigger
  //       m_robotDrive));
  // }



// Below code stops the xbox controller. In theory the doNothing thing doesn't need args but we couldn't make it work. Will refine latter
  // public void beginAutonomous() {
  //   // below does nothing
  //   m_robotDrive.setDefaultCommand(
  //     new RunCommand(() -> m_robotDrive
  //       .doNothing(-m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed),
  //       -m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed), m_driveController.getLeftBumper()),
  //       m_robotDrive));
  //   //m_robotDrive.getDefaultCommand().cancel();
  // }

  private void configureSubsystems() {

    m_PD = new PowerDistribution(1, ModuleType.kRev);

    m_driveController = new XboxController(OIConstants.kDriveControllerPort);

    m_limelight = new LimelightSubsystem();

    m_limelightTarget = new LimelightTargetSubsystem();

    m_internalBallDetector = new InternalBallDetectorSubsystem();

    m_statusLightSubsystem = new StatusLightSubsystem();

    m_ballAcquire = new BallAcquirePlanSubsystem(m_limelight, m_PD, m_statusLightSubsystem);

    m_ballShoot = new BallShooterPlanSubsystem(m_limelightTarget, m_statusLightSubsystem);
    
    m_robotDrive = new DriveSubsystem(m_ballAcquire, m_ballShoot);

    m_elevator = new ElevatorSubsystem();

    m_intakeRoller = new IntakeSubsystem();
    m_ballMover = new BallMoverSubsystem();
    m_shooter = new ShooterSubsystem();

  }


  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
      if (OIConstants.kUseAuxController) {
        // TODO - if we decide to use an aux controller then set that up
        // I believe we decided against the aux controller, but double check with Cole/Jason -AC
        return;
      }
      else {
        /* ***** --- Elevator Subsystem --- ***** */
        // Using the driver station, we know that "A" is button 1 and "Y" is button 4 (see constants)
      
        // Raise elevator
        new JoystickButton(m_driveController, OIConstants.kUpElevator)
        .whenPressed(new InstantCommand(m_elevator::upElevator, m_elevator)); // TODO - <<<>>> This needs to be a whenHeld for slewrate limiter to work
        // Stop raising elevator when button is released
        new JoystickButton(m_driveController, OIConstants.kUpElevator)
        .whenReleased(new InstantCommand(m_elevator::stopElevator, m_elevator));

        // Lower elevator
        new JoystickButton(m_driveController, OIConstants.kDownElevator)
        .whenPressed(new InstantCommand(m_elevator::downElevator, m_elevator));
        // Stop lowering elevator when button is released
        new JoystickButton(m_driveController, OIConstants.kDownElevator)
        .whenReleased(new InstantCommand(m_elevator::stopElevator, m_elevator));

        
        /* ***** --- Intake Subsystem --- ***** */
        new JoystickButton(m_driveController, OIConstants.kIntake)
        .whenPressed(new InstantCommand(m_intakeRoller::start, m_intakeRoller).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kIntake + " Pressed")));
        // When button (B) on the joystick is held, the intake motor will start. Before lowering it will say "Joystick Button (2) Pressed"
        new JoystickButton(m_driveController, OIConstants.kIntake)
        .whenReleased(new InstantCommand(m_intakeRoller::stop, m_intakeRoller).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kIntake + " Released")));
        // When button (B) on the joystick is released, the intake motor will stop. Before stopping it will say "Joystick Button (2) Released"
        
        /* ***** --- BallMover Subsystem --- ***** */
        new JoystickButton(m_driveController, OIConstants.kBallMover)
        .whenPressed(new InstantCommand(m_ballMover::start, m_ballMover).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kBallMover + " Pressed")));
        // When button (X) on the joystick is held, the ball mover will start. Before raising it will say "Joystick Button (3) Pressed"
        new JoystickButton(m_driveController, OIConstants.kBallMover)
        .whenReleased(new InstantCommand(m_ballMover::stop, m_ballMover).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kBallMover + " Released")));
        // When button (X) on the joystick is released, the feeder arm will stop raising. Before stopping it will say "Joystick Button (10) Released"
        new JoystickButton(m_driveController, OIConstants.kBallMoverReversed)
        .whenPressed(new InstantCommand(m_ballMover::reverse, m_ballMover).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kBallMoverReversed + " Pressed")));
        // When button (Back) on the joystick is pressed, the feeder will stop. Before stopping it will say "Joystick Button (10) Released"
        new JoystickButton(m_driveController, OIConstants.kBallMoverReversed)
        .whenReleased(new InstantCommand(m_ballMover::stop, m_ballMover).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kBallMoverReversed + " Released")));
        // When button (Back) on the joystick is released, the feeder arm will stop raising. Before stopping it will say "Joystick Button (10) Released"
        
  
        /* ***** --- Shooter Subsystem --- ***** */
        new JoystickButton(m_driveController, OIConstants.kShooter)
        .whenPressed(new InstantCommand(m_shooter::start, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooter + " Pressed")));
        // When the right bumper (RB) on the joystick is held, the shooter will start.
        new JoystickButton(m_driveController, OIConstants.kShooter)
        .whenReleased(new InstantCommand(m_shooter::stop, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooter + " Released")));
        // When the right bumber (RB) on the joystick is released, the shooter will stop.
        new JoystickButton(m_driveController, OIConstants.kShooterReversed)
        .whenPressed(new InstantCommand(m_shooter::reverse, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooterReversed + " Pressed")));
        // When the start button on the xbox is pressed, the shooter will reverse.
        new JoystickButton(m_driveController, OIConstants.kShooterReversed)
        .whenReleased(new InstantCommand(m_shooter::stop, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooterReversed + " Released")));
        // When the start button on the joystick is released, the shooter will stop.

        /* Ball Aquire Lighting */
        new JoystickButton(m_driveController, OIConstants.kSemiAutoBallSeek)
        .whenPressed(new InstantCommand(m_ballAcquire::lightsOn, m_ballAcquire));
        new JoystickButton(m_driveController, OIConstants.kSemiAutoBallSeek)
        .whenReleased(new InstantCommand(m_ballAcquire::lightsOff, m_ballAcquire));
      }
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
      //return new AutonomousCommand(m_robotDrive, 0.5, m_shooter, m_ballMover);
      return new SequentialCommandGroup(
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new SeekBall(m_robotDrive, m_intakeRoller, m_ballAcquire, m_limelight, 1000, 5000, m_internalBallDetector, m_PD), //Time with no ball seen before ending, Total duration
        new Target(m_robotDrive, m_ballShoot, m_limelightTarget, 1000, 3000), // Not seen timeout, total duration.
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new DriveStraightWithDelay(m_robotDrive, m_internalBallDetector, 500, .5, 0) // duration, speed, delay. 1000 worked at scrimage. keeping it at 2000 to be safe.
      );
    }

    public Command getTeleopCommand() { //Returns commands we want scheduled durring teleoperated period
      // Drive SlewRateLimiter
      SlewRateLimiter filter = new SlewRateLimiter(1.75);
      // Turn SlewRateLimiter
      SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);

      return new RunCommand(() -> m_robotDrive
      // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
        .performDrive(
          filterRotation.calculate(
            -m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kTurnLowSpeed : kTurnFullSpeed)),
          filter.calculate(
            m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kDriveLowSpeed : kDriveFullSpeed)), 
          m_driveController.getLeftBumper(), //Turns on semiautonomous ball acquire
          m_driveController.getLeftTriggerAxis() > 0.5), //Turns on semiautonomous targeter on Left Trigger
        m_robotDrive);
    }
  }

