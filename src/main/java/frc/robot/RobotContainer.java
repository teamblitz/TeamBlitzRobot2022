/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.commands.SeekBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.Target;
import frc.robot.commands.drive.AquireBallCommand;
import frc.robot.commands.drive.DriveDefaultCommand;
import frc.robot.commands.elevator.ContinuouslyRunElevatorCommand;
import frc.robot.commands.tests.DriveTest;
import frc.robot.subsystems.*;
import frc.robot.utils.ButtonBinder;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.SaitekX52Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Power Board
    private PowerDistribution powerDistribution;

    /* ***** --- Subsystems --- ***** */
    private DriveSubsystem driveSubsystem;

    private InternalBallDetectorSubsystem internalBallDetectorSubsystem;

    private StatusLightSubsystem statusLightSubsystem;

    private VisionSubsystem visionSubsystem;

    private ElevatorSubsystem elevatorSubsystem;

    private ShooterSubsystem shooterSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private BallMoverSubsystem ballMoverSubsystem;

    /* ***** --- Commands --- ***** */

    private Command intakeCommand;

    /* ***** --- Controllers --- ***** */
    private XboxController xboxController;
    private final SaitekX52Joystick saitekController;
    private final ButtonBox buttonBoard;

    // Controller Constants:
    private final double kDriveLowSpeed = 0.75;
    private final double kDriveFullSpeed = 1.0;
    private final double kDriveMinSpeed = kDriveLowSpeed / 2;
    private final double kDriveMultiplyer = kDriveFullSpeed - kDriveMinSpeed;

    private final double kTurnLowSpeed = 0.50;
    private final double kTurnFullSpeed = .60;

    // Drive SlewRateLimiter
    private final SlewRateLimiter filter = new SlewRateLimiter(1.75);
    // Turn SlewRateLimiter
    private final SlewRateLimiter filterRotation = new SlewRateLimiter(1.75);

    public RobotContainer() {
        configureSubsystems();
        buildCommands();
        setDefaultCommands();

        UsbCamera driveCamera = CameraServer.startAutomaticCapture();
        driveCamera.setResolution(320, 240);
        visionSubsystem.lightsOff(); // Turn off our lights/

        ShuffleboardTab cmdTab = Shuffleboard.getTab("Tests");
        cmdTab.add("Drive test", new DriveTest(driveSubsystem));
        cmdTab.add("Drive test rpm", new DriveTest(driveSubsystem));

        if (OIConstants.USE_XBOX_CONTROLLER)
            xboxController = new XboxController(OIConstants.DRIVE_CONTROLLER_PORT);
        else if (OIConstants.USE_SAITEK_CONTROLLER)
            saitekController = new SaitekX52Joystick(OIConstants.DRIVE_CONTROLLER_PORT);
        buttonBoard = new ButtonBox(OIConstants.BUTTON_BOX_PORT);

        configureButtonBindings();
    }

    private void setDefaultCommands() {
        // TODO: Move this into a command class.
        if (OIConstants.USE_XBOX_CONTROLLER) {
            driveSubsystem.setDefaultCommand(
                    new DriveDefaultCommand(
                            () ->
                                    filter.calculate(
                                            -xboxController.getLeftY()
                                                    * (xboxController.getRawAxis(
                                                                            OIConstants.XboxMappings
                                                                                    .OVERDRIVE
                                                                                    .value)
                                                                    < 0.5
                                                            ? kDriveLowSpeed
                                                            : kDriveFullSpeed)),
                            () ->
                                    filterRotation.calculate(
                                            xboxController.getRightX()
                                                    * (xboxController.getRawAxis(
                                                                            OIConstants.XboxMappings
                                                                                    .OVERDRIVE
                                                                                    .value)
                                                                    < 0.5
                                                            ? kTurnLowSpeed
                                                            : kTurnFullSpeed)),
                            driveSubsystem));
        } else if (OIConstants.USE_SAITEK_CONTROLLER) {
            driveSubsystem.setDefaultCommand(
                    new DriveDefaultCommand(
                            () ->
                                    filter.calculate(
                                            -saitekController.getY()
                                                    * (saitekController.getRawButton(
                                                                    SaitekX52Joystick.Button
                                                                            .kModeBlue
                                                                            .value)
                                                            ? 1.0
                                                            : saitekController.getRawButton(
                                                                            SaitekX52Joystick.Button
                                                                                    .kModeRed
                                                                                    .value)
                                                                    ? .5
                                                                    : .75)),
                            () ->
                                    filterRotation.calculate(
                                            kTurnLowSpeed
                                                    * saitekController.getRawAxis(
                                                            SaitekX52Joystick.Axis.kZRot.value)),
                            driveSubsystem));
        }
    }

    private void configureSubsystems() {

        powerDistribution = new PowerDistribution(1, ModuleType.kRev);

        internalBallDetectorSubsystem = new InternalBallDetectorSubsystem();

        statusLightSubsystem = new StatusLightSubsystem();

        visionSubsystem = new VisionSubsystem(statusLightSubsystem, powerDistribution);

        driveSubsystem = new DriveSubsystem(visionSubsystem);

        elevatorSubsystem = new ElevatorSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        ballMoverSubsystem = new BallMoverSubsystem();

        shooterSubsystem = new ShooterSubsystem();
    }

    public void buildCommands() {
        intakeCommand =
                new StartEndCommand(intakeSubsystem::start, intakeSubsystem::stop, intakeSubsystem);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (OIConstants.USE_XBOX_CONTROLLER) {

            // Create xbox button mapping

            // We can chain the methods as any command binders will return the button they were
            // called on.
            /* ***** --- Elevator Subsystem --- ***** */

            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.UP_ELEVATOR)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.UP_ELEVATOR))
                    .whenActive(
                            new InstantCommand(
                                    elevatorSubsystem::upElevator,
                                    elevatorSubsystem)) // Raise elevator
                    .whenInactive(
                            new InstantCommand(
                                    elevatorSubsystem::stopElevator,
                                    elevatorSubsystem)); // Stop raising elevator when button is
            // released

            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.DOWN_ELEVATOR)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.DOWN_ELEVATOR))
                    .whenActive(
                            new InstantCommand(
                                    elevatorSubsystem::downElevator,
                                    elevatorSubsystem)) // Lower elevator
                    .whenInactive(
                            new InstantCommand(
                                    elevatorSubsystem::stopElevator,
                                    elevatorSubsystem)); // Stop lowering elevator when button is
            // released

            /* ***** --- Intake Subsystem --- ***** */
            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.INTAKE)
                    .or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.INTAKE))
                    .whenActive(
                            new InstantCommand(
                                    intakeSubsystem::start, intakeSubsystem)) // Start intake
                    .whenInactive(
                            new InstantCommand(
                                    intakeSubsystem::stop, intakeSubsystem)); // Stop intake

            // ButtonBinder.bindButton(m_driveController, OIConstants.SEMI_AUTO_BALL_SEEK) // Enable
            // intake when we press down the SemiAutoBallSeek button
            // .whenActive(new InstantCommand(m_intakeRoller::start, m_intakeRoller)) // Start
            // intake
            // .whenInactive(new InstantCommand(m_intakeRoller::stop, m_intakeRoller)); // Stop
            // intake

            /* ***** --- BallMover Subsystem --- ***** */
            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.BALL_MOVER)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.BALL_MOVER))
                    .whenActive(
                            new InstantCommand(
                                    ballMoverSubsystem::start,
                                    ballMoverSubsystem)) // Start ball mover
                    .whenInactive(
                            new InstantCommand(
                                    ballMoverSubsystem::stop,
                                    ballMoverSubsystem)); // Stop ball mover

            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.BALL_MOVER_REVERSED)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.BALL_MOVER_REVERSED))
                    .whenActive(
                            new InstantCommand(
                                    ballMoverSubsystem::reverse,
                                    ballMoverSubsystem)) // Reverse ball mover
                    .whenInactive(
                            new InstantCommand(
                                    ballMoverSubsystem::stop,
                                    ballMoverSubsystem)); // Stop ball mover

            /* ***** --- Shooter Subsystem --- ***** */
            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.SHOOTER)
                    .or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.SHOOTER))
                    .whenActive(
                            new InstantCommand(
                                    shooterSubsystem::start, shooterSubsystem)) // Start shooter
                    .whenInactive(
                            new InstantCommand(
                                    shooterSubsystem::stop, shooterSubsystem)); // Stop shooter

            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.SHOOTER_REVERSED)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.SHOOTER_REVERSED))
                    .whenActive(
                            new InstantCommand(
                                    shooterSubsystem::reverse, shooterSubsystem)) // Reverse shooter
                    .whenInactive(
                            new InstantCommand(
                                    shooterSubsystem::stop, shooterSubsystem)); // Stop shooter

            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.SEMI_AUTO_BALL_TARGET)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard,
                                    OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_TARGET))
                    .whenActive(
                            new InstantCommand(
                                    shooterSubsystem::start, shooterSubsystem)) // Start shooter
                    .whenInactive(
                            new InstantCommand(
                                    shooterSubsystem::stop, shooterSubsystem)); // Stop shooter

            /* Ball Acquire Lighting */
            ButtonBinder.bindButton(xboxController, OIConstants.XboxMappings.SEMI_AUTO_BALL_SEEK)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_SEEK))
                    .whenActive(new InstantCommand(visionSubsystem::lightsOn)) // Lights on
                    .whenInactive(new InstantCommand(visionSubsystem::lightsOff)); // Lights off
        } else if (OIConstants.USE_SAITEK_CONTROLLER) {

            // Create saitek button mapping

            // We can chain the methods as any command binders will return the button they were
            // called on.
            /* ***** --- Elevator Subsystem --- ***** */

            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.UP_ELEVATOR)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.UP_ELEVATOR))
                    .whenActive(
                            new InstantCommand(
                                    elevatorSubsystem::upElevator,
                                    elevatorSubsystem)) // Raise elevator
                    .whenInactive(
                            new InstantCommand(
                                    elevatorSubsystem::stopElevator,
                                    elevatorSubsystem)); // Stop raising elevator when button is
            // released

            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.DOWN_ELEVATOR)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.DOWN_ELEVATOR))
                    .whenActive(
                            new InstantCommand(
                                    elevatorSubsystem::downElevator,
                                    elevatorSubsystem)) // Lower elevator
                    .whenInactive(
                            new InstantCommand(
                                    elevatorSubsystem::stopElevator,
                                    elevatorSubsystem)); // Stop lowering elevator when button is
            // released

            ButtonBinder.bindButton(
                            buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_TARGET)
                    .toggleWhenActive(new ContinuouslyRunElevatorCommand(elevatorSubsystem));

            /* ***** --- Intake Subsystem --- ***** */
            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.INTAKE)
                    .or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.INTAKE))
                    .whileActiveOnce(intakeCommand); // Start intake

            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.INTAKE_REVERSED)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.INTAKE_REVERSED))
                    .whenActive(
                            new InstantCommand(
                                    intakeSubsystem::reverse, intakeSubsystem)) // Start intake
                    .whenInactive(
                            new InstantCommand(
                                    intakeSubsystem::stop, intakeSubsystem)); // Stop intake

            ButtonBinder.bindButton(
                            saitekController, OIConstants.SaitekMappings.SEMI_AUTO_BALL_SEEK)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard,
                                    OIConstants.ButtonBoxMappings
                                            .SEMI_AUTO_BALL_SEEK)) // Enable intake when we press
                    // down
                    // the SemiAutoBallSeek button
                    .whileActiveOnce(intakeCommand); // Start intake

            /* ***** --- BallMover Subsystem --- ***** */
            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.BALL_MOVER)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.BALL_MOVER))
                    .whenActive(
                            new InstantCommand(
                                    ballMoverSubsystem::start,
                                    ballMoverSubsystem)) // Start ball mover
                    .whenInactive(
                            new InstantCommand(
                                    ballMoverSubsystem::stop,
                                    ballMoverSubsystem)); // Stop ball mover

            ButtonBinder.bindButton(
                            saitekController, OIConstants.SaitekMappings.BALL_MOVER_REVERSED)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.BALL_MOVER_REVERSED))
                    .whenActive(
                            new InstantCommand(
                                    ballMoverSubsystem::reverse,
                                    ballMoverSubsystem)) // Reverse ball mover
                    .whenInactive(
                            new InstantCommand(
                                    ballMoverSubsystem::stop,
                                    ballMoverSubsystem)); // Stop ball mover

            /* ***** --- Shooter Subsystem --- ***** */
            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.SHOOTER)
                    .or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.SHOOTER))
                    .whenActive(
                            new InstantCommand(
                                    shooterSubsystem::start, shooterSubsystem)) // Start shooter
                    .whenInactive(
                            new InstantCommand(
                                    shooterSubsystem::stop, shooterSubsystem)); // Stop shooter

            ButtonBinder.bindButton(saitekController, OIConstants.SaitekMappings.SHOOTER_REVERSED)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.SHOOTER_REVERSED))
                    .whenActive(
                            new InstantCommand(
                                    shooterSubsystem::reverse, shooterSubsystem)) // Reverse shooter
                    .whenInactive(
                            new InstantCommand(
                                    shooterSubsystem::stop, shooterSubsystem)); // Stop shooter

            //     ButtonBinder.bindButton(saitekController,
            // OIConstants.SaitekMappings.SEMI_AUTO_BALL_TARGET).or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_TARGET))
            //             .whenActive(new InstantCommand(m_shooter::start, m_shooter)) // Start
            // shooter
            //             .whenInactive(new InstantCommand(m_shooter::stop, m_shooter)); // Stop
            // shooter

            /* Ball Acquire Lighting */
            ButtonBinder.bindButton(
                            saitekController, OIConstants.SaitekMappings.SEMI_AUTO_BALL_SEEK)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_SEEK))
                    .whenActive(new InstantCommand(visionSubsystem::lightsOn)) // Lights on
                    .whenInactive(new InstantCommand(visionSubsystem::lightsOff)); // Lights off

            /* Ball Seek */
            ButtonBinder.bindButton(
                            saitekController, OIConstants.SaitekMappings.SEMI_AUTO_BALL_SEEK)
                    .or(
                            ButtonBinder.bindButton(
                                    buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_SEEK))
                    .whileActiveOnce(new AquireBallCommand(driveSubsystem, visionSubsystem));

            //     /* Auto target */
            //     ButtonBinder.bindButton(saitekController,
            // OIConstants.SaitekMappings.SEMI_AUTO_BALL_TARGET).or(ButtonBinder.bindButton(buttonBoard, OIConstants.ButtonBoxMappings.SEMI_AUTO_BALL_TARGET))
            //             .whileActiveOnce(new TargetCommand(m_robotDrive, m_vision));
        }
    }

    public Command getAutonomousCommands() { // Autonomous code goes here
        return new SequentialCommandGroup(
                new Shoot(
                        shooterSubsystem,
                        ballMoverSubsystem,
                        1000,
                        3000), // Warmup time, Total duration
                new SeekBall(
                        driveSubsystem,
                        intakeSubsystem,
                        visionSubsystem,
                        internalBallDetectorSubsystem,
                        500,
                        3000), // Time with no ball seen before ending, Total duration
                new Target(
                        driveSubsystem,
                        visionSubsystem,
                        1000,
                        3000), // Not seen timeout, total duration.
                new Shoot(
                        shooterSubsystem,
                        ballMoverSubsystem,
                        1000,
                        3000), // Warmup time, Total duration
                new DriveStraightWithDelay(
                        driveSubsystem,
                        internalBallDetectorSubsystem,
                        500,
                        -.5,
                        0) // duration, speed, delay. 1000 worked at scrimage. keeping it at 2000 to
                // be safe.
                );
    }
}
