/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallMoverSubsystemConstants;;

public class BallMoverSubsystem extends SubsystemBase {

  // Master
  CANSparkMax m_ballMoverR = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverR, MotorType.kBrushless);
  // Slave
  CANSparkMax m_ballMoverL = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverL, MotorType.kBrushless);

  public BallMoverSubsystem() {
    m_ballMoverR.restoreFactoryDefaults();
    m_ballMoverL.restoreFactoryDefaults();
    // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
    // Time to Failure Summary
    // 20A Limit - Motor survived full 220s test.
    // 40A Limit - Motor failure at approximately 27s.
    // 60A Limit - Motor failure at approximately 5.5s
    // 80A Limit* - Motor failure at approximately 2.0s
    m_ballMoverR.setSmartCurrentLimit(15);
    m_ballMoverL.setSmartCurrentLimit(15);
  }
  // Enables BallMover Wheels
  public void start() {
    System.out.println("BallMoverSubsystem::start");
    m_ballMoverR.set(0.45);
    m_ballMoverL.set(-0.45);
  }

  // This could reverse BallMover Wheels
  public void reverse(){
    m_ballMoverR.set(0.45);
    m_ballMoverL.set(-0.45);
   }

  // Disable BallMover Wheels
  public void stop() {
    System.out.println("BallMoverSubsystem::stop");
    m_ballMoverR.set(0.0);
    m_ballMoverL.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
