/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {
  private static final int 
  leftDeviceID = Constants.DriveConstants.kLeftMasterPort, 
  leftSlaveDeviceID = Constants.DriveConstants.kLeftSlavePort;  // Should be 5 master, 6 slave

  private static final int 
  rightDeviceID = Constants.DriveConstants.kRightMasterPort, 
  rightSlaveDeviceID = Constants.DriveConstants.kRightSlavePort;  // Should be 4 master, 3 slave
  private CANSparkMax m_leftMotor, m_leftMotorSlave;
  private CANSparkMax m_rightMotor, m_rightMotorSlave;
  private BallAcquirePlanSubsystem m_ballAcquire;
  private BallShooterPlanSubsystem m_ballShoot;

  private DifferentialDrive m_drive;
 
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(BallAcquirePlanSubsystem ballAcquire, BallShooterPlanSubsystem ballShoot) {
    m_ballAcquire = ballAcquire;
    m_ballShoot = ballShoot;
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
 
  // right side
  m_rightMotor.follow(ExternalFollower.kFollowerDisabled, 0);
  // Enable for dual motors
  m_rightMotorSlave.follow(ExternalFollower.kFollowerSparkMax, rightDeviceID);

  m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor); 
  }
    
  @Override
  public void periodic() {
    
    }


  /**
   * Drives the robot using one of several control methods.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

    public void doNothing(final double fwd, final double rot, final boolean semiAutonomousState) {
    }

    public void seed(){ //Feeds our moter s
      m_drive.feed();
    }

    public void performDrive(final double fwd, final double rot, final boolean semiAutonomousState, final boolean targetingState) {
  
      // decide who is in control and execute their drive operations
      if(semiAutonomousState)
      {
        arcadeDrive(m_ballAcquire.getRot(), m_ballAcquire.getFwd()); // Again, our arcade drive is reversed for some reason, so we reverse this.
      }
      else if (targetingState){
        arcadeDrive(m_ballShoot.getRot(), m_ballShoot.getFwd());
      }
      else
      {
        arcadeDrive(fwd, rot);
      }
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
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }
  
  public void tankDrive(final double leftSpeed, final double rightSpeed) {
    // Instead of calling tankDrive, call set(ControlMode.Velocity, ...) on each master motor directly.
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
}