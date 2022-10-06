/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.commands.SeekBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.Target;
import frc.robot.commands.tests.DriveTest;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ButtonBinder;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.SaitekX52Joystick;
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

  private InternalBallDetectorSubsystem m_internalBallDetector; 

  private StatusLightSubsystem m_statusLightSubsystem;

  private VisionSubsystem m_vision;

  private ElevatorSubsystem m_elevator;

  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intakeRoller;

  private BallMoverSubsystem m_ballMover;

  /* ***** --- Commands --- ***** */

  private Command intakeCommand;


  /* ***** --- Controllers --- ***** */
  private XboxController m_xboxController;
  private SaitekX52Joystick m_saitekController;
  private ButtonBox m_buttonBoard;

  // Controller Constants: 
  private final double kDriveLowSpeed = 0.75;
  private final double kDriveFullSpeed = 1.0;
  private final double kDriveMinSpeed = kDriveLowSpeed/2;
  private final double kDriveMultiplyer = kDriveFullSpeed - kDriveMinSpeed;
  

  

  private final double kTurnLowSpeed = 0.45;
  private final double kTurnFullSpeed = .60;

  // Drive SlewRateLimiter
  private final SlewRateLimiter filter = new SlewRateLimiter(1.75);
  // Turn SlewRateLimiter
  private final SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);

  public RobotContainer() {
    configureSubsystems();
    buildCommands();
    setDefaultCommands();
    CameraServer.startAutomaticCapture();
    m_vision.lightsOff(); // Turn off our lights/
    
    SmartDashboard.putData("Drive test", new DriveTest(m_robotDrive));
    
    if (OIConstants.useXboxController) m_xboxController = new XboxController(OIConstants.kDriveControllerPort);
    else if (OIConstants.useSaitekController) m_saitekController = new SaitekX52Joystick(OIConstants.kDriveControllerPort);
    m_buttonBoard = new ButtonBox(OIConstants.kButtonBoxPort);
    
    configureButtonBindings();
  
  }

  private void setDefaultCommands() {
    // Set defalut command for drive

    if (OIConstants.useXboxController) {
      m_robotDrive.setDefaultCommand(
        new RunCommand(() -> m_robotDrive
        // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
        .performDrive(
          filter.calculate(
            -m_xboxController.getLeftY() * (m_xboxController.getRawAxis(OIConstants.XboxMappings.kOverdrive.value) < 0.5 ? kDriveLowSpeed : kDriveFullSpeed)), 
          filterRotation.calculate(
            m_xboxController.getRightX() * (m_xboxController.getRawAxis(OIConstants.XboxMappings.kOverdrive.value) < 0.5 ? kTurnLowSpeed : kTurnFullSpeed)),
          m_xboxController.getRawButton(OIConstants.XboxMappings.kSemiAutoBallSeek.value), //Turns on semiautonomous ball acquire
          m_xboxController.getRawAxis(OIConstants.XboxMappings.kSemiAutoBallTarget.value) > 0.5), //Turns on semiautonomous targeter on Left Trigger
        m_robotDrive).withName("DriveDefalutCommand"));
    } else if (OIConstants.useSaitekController) {
      
      m_robotDrive.setDefaultCommand(
        new RunCommand(() -> m_robotDrive
        // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
        .performDrive(
          filter.calculate(
            -m_saitekController.getY() * (((-m_saitekController.getRawAxis(SaitekX52Joystick.Axis.kThrotle.value)+1)/2) * kDriveMultiplyer + kDriveMinSpeed)),
          filterRotation.calculate(
            kTurnLowSpeed * m_saitekController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value)),
          false, //Turns on semiautonomous ball acquire
          false), //Turns on semiautonomous targeter on Left Trigger
        m_robotDrive).withName("DriveDefalutCommand"));    }
  }


  private void configureSubsystems() {

    m_PD = new PowerDistribution(1, ModuleType.kRev);

    m_internalBallDetector = new InternalBallDetectorSubsystem();

    m_statusLightSubsystem = new StatusLightSubsystem();

    m_vision = new VisionSubsystem(m_statusLightSubsystem, m_PD);
    
    m_robotDrive = new DriveSubsystem(m_vision);

    m_elevator = new ElevatorSubsystem();

    m_intakeRoller = new IntakeSubsystem();

    m_ballMover = new BallMoverSubsystem();

    m_shooter = new ShooterSubsystem();

  }

  public void buildCommands () {
    intakeCommand = new StartEndCommand(m_intakeRoller::start, m_intakeRoller::stop, m_intakeRoller);

  }


  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
      if (OIConstants.useXboxController) {

        // Create saitek button mapping

        // We can chain the the methods as any command binders will return the button they were called on.
        /* ***** --- Elevator Subsystem --- ***** */
      
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kUpElevator).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kUpElevator))
        .whenActive(new InstantCommand(m_elevator::upElevator, m_elevator)) // Raise elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop raising elevator when button is released

        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kDownElevator).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kDownElevator))
        .whenActive(new InstantCommand(m_elevator::downElevator, m_elevator)) // Lower elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop lowering elevator when button is released

        /* ***** --- Intake Subsystem --- ***** */
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kIntake).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kIntake))
        .whenActive(new InstantCommand(m_intakeRoller::start, m_intakeRoller)) // Start intake
        .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop intake
        
        // ButtonBinder.bindButton(m_driveController, OIConstants.kSemiAutoBallSeek) // Enable intake when we press down the SemiAutoBallSeek button
        // .whenActive(new InstantCommand(m_intakeRoller::start, m_intakeRoller)) // Start intake
        // .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop intake
        
        /* ***** --- BallMover Subsystem --- ***** */
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kBallMover).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kBallMover))
        .whenActive(new InstantCommand(m_ballMover::start, m_ballMover)) // Start ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kBallMoverReversed).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kBallMoverReversed))
        .whenActive(new InstantCommand(m_ballMover::reverse, m_ballMover)) // Reverse ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
  
        /* ***** --- Shooter Subsystem --- ***** */
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kShooter).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kShooter))
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kShooterReversed).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kShooterReversed))
        .whenActive(new InstantCommand(m_shooter::reverse, m_shooter)) // Reverse shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kSemiAutoBallTarget).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kSemiAutoBallTarget))
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        /* Ball Aquire Lighting */
        ButtonBinder.bindButton(m_xboxController, OIConstants.XboxMappings.kSemiAutoBallSeek).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kSemiAutoBallSeek))
        .whenActive(new InstantCommand(m_vision::lightsOn)) // Lights on
        .whenInactive(new InstantCommand(m_vision::lightsOff)); // Lights off
      } else if (OIConstants.useSaitekController) {
        
        // Create xbox button mapping

        // We can chain the the methods as any command binders will return the button they were called on.
        /* ***** --- Elevator Subsystem --- ***** */
      
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kUpElevator).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kUpElevator))
        .whenActive(new InstantCommand(m_elevator::upElevator, m_elevator)) // Raise elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop raising elevator when button is released

        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kDownElevator).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kDownElevator))
        .whenActive(new InstantCommand(m_elevator::downElevator, m_elevator)) // Lower elevator
        .whenInactive(new InstantCommand(m_elevator::stopElevator, m_elevator)); // Stop lowering elevator when button is released

        /* ***** --- Intake Subsystem --- ***** */
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kIntake).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kIntake))
        .whileActiveOnce(intakeCommand); // Start intake

        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kIntakeReversed).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kIntakeReversed))
        .whenActive(new InstantCommand(m_intakeRoller::reverse, m_intakeRoller)) // Start intake
        .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop intake
        
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kSemiAutoBallSeek).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kSemiAutoBallSeek)) // Enable intake when we press down the SemiAutoBallSeek button
        .whileActiveOnce(intakeCommand); // Start intake
        
        /* ***** --- BallMover Subsystem --- ***** */
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kBallMover).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kBallMover))
        .whenActive(new InstantCommand(m_ballMover::start, m_ballMover)) // Start ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kBallMoverReversed).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kBallMoverReversed))
        .whenActive(new InstantCommand(m_ballMover::reverse, m_ballMover)) // Reverse ball mover
        .whenInactive(new InstantCommand(m_ballMover::stop, m_ballMover)); // Stop ball mover
        
  
        /* ***** --- Shooter Subsystem --- ***** */
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kShooter).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kShooter))
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kShooterReversed).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kShooterReversed))
        .whenActive(new InstantCommand(m_shooter::reverse, m_shooter)) // Reverse shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kSemiAutoBallTarget).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kSemiAutoBallTarget))
        .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start shooter
        .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop shooter

        /* Ball Aquire Lighting */
        ButtonBinder.bindButton(m_saitekController, OIConstants.SaitekMappings.kSemiAutoBallSeek).or(ButtonBinder.bindButton(m_buttonBoard, OIConstants.ButtonBoxMappings.kSemiAutoBallSeek))
        .whenActive(new InstantCommand(m_vision::lightsOn)) // Lights on
        .whenInactive(new InstantCommand(m_vision::lightsOff)); // Lights off
      }
    }

    public Command getAutonomousCommands() { // Autonomous code goes here
      return new SequentialCommandGroup(
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new SeekBall(m_robotDrive, m_intakeRoller, m_vision, m_internalBallDetector, 500, 3000), //Time with no ball seen before ending, Total duration
        new Target(m_robotDrive, m_vision, 1000, 3000), // Not seen timeout, total duration.
        new Shoot(m_shooter, m_ballMover, 1000, 3000), //Warmup time, Total duration
        new DriveStraightWithDelay(m_robotDrive, m_internalBallDetector, 500, .5, 0) // duration, speed, delay. 1000 worked at scrimage. keeping it at 2000 to be safe.
      );
    }

  }

