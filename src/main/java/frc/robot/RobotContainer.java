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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.commands.SeekBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.Target;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ButtonBinder;
import frc.robot.subsystems.BallMoverSubsystem;
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

  /* ***** --- Subsystems --- ***** */
  private DriveSubsystem m_robotDrive;

  // private LimelightSubsystem m_limelight;

  // private LimelightTargetSubsystem m_limelightTarget;

  private InternalBallDetectorSubsystem m_internalBallDetector; 

  private StatusLightSubsystem m_statusLightSubsystem;

  // private BallAcquirePlanSubsystem m_ballAcquire; // Deprecated. Should be safe to remove.

  // private BallShooterPlanSubsystem m_ballShoot; // Deprecated. Should be safe to remove.

  private VisionSubsystem m_vision;

  private ElevatorSubsystem m_elevator;

  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intakeRoller;

  private BallMoverSubsystem m_ballMover;

  /* ***** --- Controllers --- ***** */
  private XboxController m_driveController;

  // Controller Constants: 
  // TODO - <<<>>> Move to Constants
  private final double kDriveLowSpeed = 0.75;
  private final double kDriveFullSpeed = 1.0;

  private final double kTurnLowSpeed = 0.45;
  private final double kTurnFullSpeed = .60;

  // Drive SlewRateLimiter
  SlewRateLimiter filter = new SlewRateLimiter(1.75);
  // Turn SlewRateLimiter
  SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);

  public RobotContainer() {

    configureSubsystems();
    configureButtonBindings();
    setDefaultCommands();
    CameraServer.startAutomaticCapture();
    m_vision.lightsOff(); // Turn off our lights/
  }


  private void setDefaultCommands() {
    // Set defalut command for drive
    m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
      .performDrive(
        filterRotation.calculate(
          -m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdrive.value) < 0.5 ? kTurnLowSpeed : kTurnFullSpeed)),
        filter.calculate(
          m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdrive.value) < 0.5 ? kDriveLowSpeed : kDriveFullSpeed)), 
        m_driveController.getRawButton(OIConstants.kSemiAutoBallSeek.value), //Turns on semiautonomous ball acquire
        m_driveController.getRawAxis(OIConstants.kSemiAutoBallTarget.value) > 0.5), //Turns on semiautonomous targeter on Left Trigger
      m_robotDrive).withName("DriveDefalutCommand"));
  }


  private void configureSubsystems() {

    m_PD = new PowerDistribution(1, ModuleType.kRev);

    m_driveController = new XboxController(OIConstants.kDriveControllerPort);

    // m_limelight = new LimelightSubsystem();

    // m_limelightTarget = new LimelightTargetSubsystem();

    m_internalBallDetector = new InternalBallDetectorSubsystem();

    m_statusLightSubsystem = new StatusLightSubsystem();

    // m_ballAcquire = new BallAcquirePlanSubsystem(m_limelight, m_PD, m_statusLightSubsystem);

    // m_ballShoot = new BallShooterPlanSubsystem(m_limelightTarget, m_statusLightSubsystem);

    m_vision = new VisionSubsystem(m_statusLightSubsystem, m_PD);
    
    m_robotDrive = new DriveSubsystem(m_vision);

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
        return;
      }
      else {
        // We can chain the the methods as any command binders will return the button they were called on.
        /* ***** --- Elevator Subsystem --- ***** */
      
        ButtonBinder.bindButton(m_driveController, OIConstants.kUpElevator)
        .whenActive(new InstantCommand(m_elevator::upElevator, m_elevator)) // Raise elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop raising elevator when button is released

        ButtonBinder.bindButton(m_driveController, OIConstants.kDownElevator)
        .whenActive(new InstantCommand(m_elevator::downElevator, m_elevator)) // Lower elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop lowering elevator when button is released

        /* ***** --- Intake Subsystem --- ***** */
        ButtonBinder.bindButton(m_driveController, OIConstants.kIntake)
        .whenActive(new InstantCommand(m_intakeRoller::start, m_intakeRoller)) // Start intake
        .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop intake
        
        ButtonBinder.bindButton(m_driveController, OIConstants.kSemiAutoBallSeek) // Enable intake when we press down the SemiAutoBallSeek button
        .whenActive(new InstantCommand(m_intakeRoller::start, m_intakeRoller)) // Start intake
        .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop intake
        
        /* ***** --- BallMover Subsystem --- ***** */
        ButtonBinder.bindButton(m_driveController, OIConstants.kBallMover)
        .whenActive(new InstantCommand(m_ballMover::start, m_ballMover)) // Start ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
        ButtonBinder.bindButton(m_driveController, OIConstants.kBallMoverReversed)
        .whenActive(new InstantCommand(m_ballMover::reverse, m_ballMover)) // Reverse ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
  
        /* ***** --- Shooter Subsystem --- ***** */
        ButtonBinder.bindButton(m_driveController, OIConstants.kShooter)
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_driveController, OIConstants.kShooterReversed)
        .whenActive(new InstantCommand(m_shooter::reverse, m_shooter)) // Reverse shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_driveController, OIConstants.kSemiAutoBallTarget)
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        /* Ball Aquire Lighting */
        ButtonBinder.bindButton(m_driveController, OIConstants.kSemiAutoBallSeek)
        .whenActive(new InstantCommand(m_vision::lightsOn)) // Lights on
        .whenInactive(new InstantCommand(m_vision::lightsOff)); // Lights off
      }
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
      return new SequentialCommandGroup(
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new SeekBall(m_robotDrive, m_intakeRoller, m_vision, m_internalBallDetector, 500, 3000), //Time with no ball seen before ending, Total duration
        new Target(m_robotDrive, m_vision, 1000, 3000), // Not seen timeout, total duration.
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new DriveStraightWithDelay(m_robotDrive, m_internalBallDetector, 500, .5, 0) // duration, speed, delay. 1000 worked at scrimage. keeping it at 2000 to be safe.
      );
    }

  }

