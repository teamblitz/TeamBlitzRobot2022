/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StatusManager;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax leftMotor, leftMotorSlave, rightMotor, rightMotorSlave;
    private final VisionSubsystem visionSubsystem;

    private final DifferentialDrive drive;

    private final AHRS gyro = new AHRS();

    private final NetworkTableEntry kPValue;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        // *********** PUT NON-TUNABLE PARAMETERS BELOW THIS LINE **********
        /*
         SPARK MAX controllers are initialized over CAN by constructing a CANSparkMax object

         The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
         first parameter

         The motor type is passed as the second parameter. Motor type can either be:
          com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
          com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed

         The example below initializes four brushless motors with CAN IDs 1 and 2. Change
         these parameters to match your setup
        */
        leftMotor = new CANSparkMax(LEFT_MASTER_PORT, MotorType.kBrushless);
        leftMotorSlave = new CANSparkMax(LEFT_SLAVE_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(RIGHT_MASTER_PORT, MotorType.kBrushless);
        rightMotorSlave = new CANSparkMax(RIGHT_SLAVE_PORT, MotorType.kBrushless);

        /*
         The RestoreFactoryDefaults method can be used to reset the configuration parameters
         in the SPARK MAX to their factory default state. If no argument is passed, these
         parameters will not persist between power cycles
        */
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotorSlave.restoreFactoryDefaults();
        rightMotorSlave.restoreFactoryDefaults();

        // left side
        // setup slave relationship on motors on same side
        leftMotor.follow(ExternalFollower.kFollowerDisabled, 0);
        leftMotorSlave.follow(ExternalFollower.kFollowerSparkMax, LEFT_MASTER_PORT);

        // If the robot doesn't go straight when we tell it to, set this to true. I have no clue why
        // it stopped needing to be inverted, but it works now, and I am afraid to change it.
        leftMotor.setInverted(false);
        // right side
        rightMotor.follow(ExternalFollower.kFollowerDisabled, 0);
        rightMotorSlave.follow(ExternalFollower.kFollowerSparkMax, RIGHT_MASTER_PORT);

        drive = new DifferentialDrive(leftMotor, rightMotor);

        // Add the spark objects to the status manager.
        StatusManager status = StatusManager.getInstance();
        status.addMotor(leftMotor, "leftDriveM");
        status.addMotor(rightMotor, "rightDriveM");
        status.addMotor(leftMotorSlave, "leftDriveS");
        status.addMotor(rightMotorSlave, "rightDriveS");

        ShuffleboardTab tab = Shuffleboard.getTab("Motors");
        tab.addNumber("LeftDriveM", () -> leftMotor.getEncoder().getVelocity());
        tab.addNumber("LeftDriveS", () -> leftMotorSlave.getEncoder().getVelocity());
        tab.addNumber("RightDriveM", () -> rightMotor.getEncoder().getVelocity());
        tab.addNumber("RightDriveS", () -> rightMotorSlave.getEncoder().getVelocity());

        Shuffleboard.getTab("Drive").addBoolean("Driving Straight ", () -> wasDrivingStraight);
        Shuffleboard.getTab("Drive").addNumber("Driving Straight Angle", () -> wantedAngle);
        Shuffleboard.getTab("Drive").addNumber("Gyro Angle", gyro::getAngle);

        kPValue = Shuffleboard.getTab("Drive").add("P", .009).getEntry();
    }

    @Override
    public void periodic() {
        drive.feed();
    }

    /**
     * Drives the robot using one of several control methods.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void performDrive(
            final double fwd,
            final double rot,
            final boolean semiAutonomousState,
            final boolean targetingState) {

        // decide who is in control and execute their drive operations
        if (semiAutonomousState) {
            arcadeDrive(
                    visionSubsystem.ballAcquirePlan.getFwd(),
                    visionSubsystem.ballAcquirePlan.getRot(),
                    false);
            visionSubsystem.ballAcquirePlan.statusLights();
        } else if (targetingState) {
            arcadeDrive(
                    visionSubsystem.ballShooterPlan.getFwd(),
                    visionSubsystem.ballShooterPlan.getRot(),
                    false);
            visionSubsystem.ballShooterPlan.statusLights();
        } else {
            arcadeDrive(fwd, rot, true);
            visionSubsystem.statusLightsOff(); // Turn off status lights
        }
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot, boolean squareInputs) {
        rot = MathUtil.applyDeadband(rot, 0.005);

        if (!wasDrivingStraight && rot == 0 && fwd != 0) {
            wasDrivingStraight = true;
            wantedAngle = gyro.getAngle();
        } else if (rot != 0 || fwd == 0) {
            wasDrivingStraight = false;
        }
        if (wasDrivingStraight) {
            drive_straight_gyro(MathUtil.clamp(fwd, -1, 1));
            return;
        }
        drive.arcadeDrive(MathUtil.clamp(fwd, -1, 1), MathUtil.clamp(rot, -1, 1), squareInputs);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(rightVolts);
        drive.feed();
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    private double kP = 0.05;
    private boolean wasDrivingStraight;
    private double wantedAngle;

    public void drive_straight_gyro(double speed) {
        kP = kPValue.getDouble(.009);
        double error = wantedAngle - gyro.getAngle(); // Our target angle is zero
        double turn_power = kP * error;
        drive.arcadeDrive(speed, MathUtil.clamp(turn_power, -0.1, .1), false);
    }
}
