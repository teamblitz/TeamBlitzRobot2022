/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallAcquirePlanSubsystem;
import frc.robot.subsystems.BallMoverSubsystem;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  // Chassis drive subsystem:
  private DriveSubsystem m_robotDrive;

  // Limelight subsystem:
  private LimelightSubsystem m_limelight;

  // Ball Acquire subsystem:
  private BallAcquirePlanSubsystem m_ballAcquire;

  // Elevator Subsystem:
  private ElevatorSubsystem m_elevator;

  // Shooter Subsystem
  private ShooterSubsystem m_shooter;

  // Controllers:
  private XboxController m_driveController;


  // Controller Constants:
  private final boolean kUseTankDrive = false;

  private IntakeSubsystem m_intakeRoller;
  private BallMoverSubsystem m_ballMover;

  private final double kLowSpeed = 0.75;
  private final double kFullSpeed = 1.0;

  public RobotContainer() {

    configureSubsystems();
    configureButtonBindings();
    
    /** SlewRateLimiter
    * Creates a SlewRateLimiter that limits the rate of change of the signal to 1.75 units per second for forward and backward
    * Essemtaly stoping jerking of the robot during arcade drive
    * Do Not delete unless removed below
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
    }

    else { //perform driving using one of several methods
      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive

      // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
        .performDrive(filter.calculate(-m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
        filterRotation.calculate(-m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)), m_driveController.getLeftBumper()),
        m_robotDrive));
    }
  }

  private void configureSubsystems() {

    m_driveController = new XboxController(OIConstants.kDriveControllerPort);

    m_limelight = new LimelightSubsystem();

    m_ballAcquire = new BallAcquirePlanSubsystem(m_limelight);
    
    m_robotDrive = new DriveSubsystem(m_ballAcquire);

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
        .whenPressed(new InstantCommand(m_elevator::upElevator, m_elevator));
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
  
        /* ***** --- Shooter Subsystem --- ***** */
        new JoystickButton(m_driveController, OIConstants.kShooter)
        .whenPressed(new InstantCommand(m_shooter::start, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooter + " Pressed")));
        // When the right bumper (RB) on the joystick is held, the shooter will start.
        new JoystickButton(m_driveController, OIConstants.kShooter)
        .whenReleased(new InstantCommand(m_shooter::stop, m_shooter).beforeStarting(() -> System.out.println("Joystick Button " + OIConstants.kShooter + " Released")));
        // When the right bumber (RB) on the joystick is released, the shooter will stop.
      }
    }

    public Command getAutonomousCommand() {
      return new DriveStraightWithDelay(m_robotDrive, 5000, 0.5, 0, m_shooter, 2000, 0);
    }
  }

