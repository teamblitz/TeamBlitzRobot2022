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
    }

    else { //arcadeDrive
      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
      // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
      .arcadeDrive(filter.calculate(-m_driveController.getRightX() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed)),
      filterRotation.calculate(-m_driveController.getLeftY() * (m_driveController.getRawAxis(OIConstants.kOverdriveRightTriggerAxis) < 0.5 ? kLowSpeed : kFullSpeed))),
      m_robotDrive));
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
  }

