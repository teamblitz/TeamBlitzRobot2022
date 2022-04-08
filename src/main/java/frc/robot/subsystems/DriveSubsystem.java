/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.networktables.NetworkTableEntry;

// import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /* Gyro configuration*/
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
  /* Odometry class for tracking robot pose*/
  private final DifferentialDriveOdometry m_odometry;
  /* Master Talons */
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(Constants.DriveConstants.kLeftMasterPort);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(Constants.DriveConstants.kRightMasterPort);
  /* Slave Talons */
  private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(Constants.DriveConstants.kLeftSlavePort);
  private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(Constants.DriveConstants.kRightSlavePort);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
 
  //NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  //NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_leftSlave.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    // Peak and Nominal Output
    m_leftMaster.configPeakOutputForward(0.8);
    m_rightMaster.configPeakOutputForward(0.8);
    m_leftMaster.configNominalOutputForward(0.1);
    m_rightMaster.configNominalOutputForward(0.1);

    // Current Limits
    double kStatorCurrentLimit = 35;
    double kStatorTriggerThreshold = 40;
    double kStatorTriggerThresholdTime = 1.0;
    double kSupplyCurrentLimit = 35;
    double kSupplyTriggerThreshold = 40;
    double kSupplyTriggerThresholdTime = 0.5;

    m_leftMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kStatorCurrentLimit, kStatorTriggerThreshold, kStatorTriggerThresholdTime));
    m_leftMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kSupplyCurrentLimit, kSupplyTriggerThreshold, kSupplyTriggerThresholdTime));
    m_rightMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kStatorCurrentLimit, kStatorTriggerThreshold, kStatorTriggerThresholdTime));
    m_rightMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kSupplyCurrentLimit, kSupplyTriggerThreshold, kSupplyTriggerThresholdTime));
    m_leftSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kStatorCurrentLimit, kStatorTriggerThreshold, kStatorTriggerThresholdTime));
    m_leftSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kSupplyCurrentLimit, kSupplyTriggerThreshold, kSupplyTriggerThresholdTime));
    m_rightSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kStatorCurrentLimit, kStatorTriggerThreshold, kStatorTriggerThresholdTime));
    m_rightSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kSupplyCurrentLimit, kSupplyTriggerThreshold, kSupplyTriggerThresholdTime));


    // Neutral Mode
    m_leftMaster.setNeutralMode(NeutralMode.Coast);
    m_rightMaster.setNeutralMode(NeutralMode.Coast);
    m_leftSlave.setNeutralMode(NeutralMode.Coast);
    m_rightSlave.setNeutralMode(NeutralMode.Coast);

    //Ramping Motor Speed (this is sketchy and should still be looked into... should be easier on sparkmax controllers)
    //RevRobotics app has a ramp configuration in motor controller settings.
    // m_leftMaster.configOpenloopRamp(0.25); // Best set to .25 or .3
    // m_rightMaster.configOpenloopRamp(0.25); //Best set to .25 or .3

    // *********** PUT NON-TUNABLE PARAMETERS BELOW THIS LINE **********

    /**
    * Take our extra motor controllers and have them
    * follow the Talons updated in arcadeDrive
    */
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    /**
    * Drive robot forward and make sure all motors spin the correct way.
    * Toggle booleans accordingly....
    */
    m_leftMaster.setInverted(TalonFXInvertType.Clockwise);          // <<<<<< Adjust this until robot drives forward when stick is forward
    m_rightMaster.setInverted(TalonFXInvertType.CounterClockwise);  // <<<<<< Adjust this until robot drives forward when stick is forward
    m_leftSlave.setInverted(InvertType.FollowMaster);
    m_rightSlave.setInverted(InvertType.FollowMaster);

    /* diff drive assumes (by default) that
      right side must be negative to move forward.
      Change to 'false' so positive/green-LEDs moves robot forward
    */
    m_rightMaster.setInverted(false); // do not change this <<<>>> THIS LOOKS SUS -CXH

    // Sets the distance per pulse for the encoders
    //m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_gyro.getRotation2d();
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse,
    m_rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse);

    m_drive.feed();
   
    // var translation = m_odometry.getPoseMeters().getTranslation();
    //m_xEntry.setNumber(translation.getX());
    //m_yEntry.setNumber(translation.getY());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
 public void arcadeDrive(final double fwd, final double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void tankDrive(final double leftSpeed, final double rightSpeed) {
    // Instead of calling tankDrive, call set(ControlMode.Velocity, ...) on each master motor directly.
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    // return m_gyro.getRotation2d().getDegrees();
    return Math.IEEEremainder(m_gyro.getRotation2d().getDegrees(), 360) * (Constants.GyroConstants.kGyroReversed ? -1.0 : 1.0);
  }

   /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(stepsPerDecisecToMetersPerSec(m_leftMaster.getSelectedSensorVelocity()),
    stepsPerDecisecToMetersPerSec(m_rightMaster.getSelectedSensorVelocity()));
  }

  public static double stepsToMeters(double d) {
    return (Constants.DriveConstants.KWheelDiameterMeters / 2048) * d;
  }

  public static double stepsPerDecisecToMetersPerSec(double d) {
    return stepsToMeters(d * 10);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftMaster.setSelectedSensorPosition(0, 0, 10);
    m_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
    m_rightMaster.setSelectedSensorPosition(0, 0, 10);
    m_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  /*
    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */

  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */

  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.GyroConstants.kGyroReversed ? -1.0 : 1.0);
  }
}