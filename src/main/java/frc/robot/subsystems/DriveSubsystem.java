/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StatusManager;


public class DriveSubsystem extends SubsystemBase {
  private final int 
    leftDeviceID = Constants.DriveConstants.kLeftMasterPort, 
    leftSlaveDeviceID = Constants.DriveConstants.kLeftSlavePort,  // Should be 6 master, 5 slave
    rightDeviceID = Constants.DriveConstants.kRightMasterPort, 
    rightSlaveDeviceID = Constants.DriveConstants.kRightSlavePort;  // Should be 4 master, 3 slave
  
  private final CANSparkMax m_leftMotor, m_leftMotorSlave;
  private final CANSparkMax m_rightMotor, m_rightMotorSlave;
  private final VisionSubsystem m_vision;

  private final DifferentialDrive m_drive;

  private final StatusManager status = StatusManager.getInstance();

  private final AHRS m_gyro = new AHRS();

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(VisionSubsystem vision) {
    m_vision = vision;
    // *********** PUT NON-TUNABLE PARAMETERS BELOW THIS LINE **********
    /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
  m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
  m_leftMotorSlave = new CANSparkMax(leftSlaveDeviceID, MotorType.kBrushless);
  m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
  m_rightMotorSlave = new CANSparkMax(rightSlaveDeviceID, MotorType.kBrushless);

  /**
   * The RestoreFactoryDefaults method can be used to reset the configuration parameters
   * in the SPARK MAX to their factory default state. If no argument is passed, these
   * parameters will not persist between power cycles
   */
  m_leftMotor.restoreFactoryDefaults();
  m_rightMotor.restoreFactoryDefaults();
  m_leftMotorSlave.restoreFactoryDefaults();
  m_rightMotorSlave.restoreFactoryDefaults();

  
  // left side
  // setup slave relationship on motors on same side
  m_leftMotor.follow(ExternalFollower.kFollowerDisabled, 0);
  // Enable for dual motors
  m_leftMotorSlave.follow(ExternalFollower.kFollowerSparkMax, leftDeviceID);
  
  m_leftMotor.setInverted(false);
  // right side
  m_rightMotor.follow(ExternalFollower.kFollowerDisabled, 0);
  // Enable for dual motors
  m_rightMotorSlave.follow(ExternalFollower.kFollowerSparkMax, rightDeviceID);

  
  
  m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor); 
  
  
  status.addMotor(m_leftMotor, "leftDriveM");
  status.addMotor(m_rightMotor, "rightDriveM");
  status.addMotor(m_leftMotorSlave, "leftDriveS");
  status.addMotor(m_rightMotorSlave, "rightDriveS");
  ShuffleboardTab tab = Shuffleboard.getTab("Motors");
  tab.addNumber("LeftDriveM", ()->m_leftMotor.getEncoder().getVelocity());
  tab.addNumber("LeftDriveS", ()->m_leftMotorSlave.getEncoder().getVelocity());
  tab.addNumber("RightDriveM", ()->m_rightMotor.getEncoder().getVelocity());
  tab.addNumber("RightDriveS", ()->m_rightMotorSlave.getEncoder().getVelocity());

  Shuffleboard.getTab("Drive").addBoolean("Driving Straight ", ()->wasDrivingStraight);
  Shuffleboard.getTab("Drive").addNumber("Driving Straight Angle", ()->wantedAngle);

  
}

  @Override
  public void periodic() {
    m_drive.feed();
  }

  /**
   * Drives the robot using one of several control methods.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

  public void performDrive(final double fwd, final double rot, final boolean semiAutonomousState, final boolean targetingState) {

    // decide who is in control and execute their drive operations
    if(semiAutonomousState)
    {
      arcadeDrive(m_vision.ballAcquirePlan.getFwd(), m_vision.ballAcquirePlan.getRot(), false); // Again, our arcade drive is reversed for some reason, so we reverse this.
      m_vision.ballAcquirePlan.statusLights();
    }
    else if (targetingState){
      arcadeDrive(m_vision.ballShooterPlan.getFwd(), m_vision.ballShooterPlan.getRot(), false);
      m_vision.ballShooterPlan.statusLights();
    }
    else
    {
      arcadeDrive(fwd, rot, true);
      m_vision.statusLightsOff(); // Turn off status lights
    }
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(final double fwd, final double rot, boolean squareInputs) {
      if (!wasDrivingStraight && rot == 0) {
        wasDrivingStraight = true;
        wantedAngle = m_gyro.getAngle();
      } else if (rot != 0) {
        wasDrivingStraight = false;
        return;
      }
      drive_straight_gyro(MathUtil.clamp(fwd, -1, 1));
    }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }
  
  public void tankDrive(final double leftSpeed, final double rightSpeed) {
    // Instead of calling tankDrive, call set(ControlMode.Velocity, ...) on each master motor directly.
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  private double kP = 0.05;
  private boolean wasDrivingStraight;
  private double wantedAngle;

  public void drive_straight_gyro(double speed) {
    double error = wantedAngle-m_gyro.getAngle();  // Our target angle is zero
    double turn_power = kP * error;
    arcadeDrive(speed, -turn_power, false);
  }
}