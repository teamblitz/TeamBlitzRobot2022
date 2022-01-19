/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.FeederSubsystemConstants;

// /**
//  * Add your docs here.
//   */
// public class FeederArmSubsystem extends SubsystemBase {
//   CANSparkMax m_intakeArm = new CANSparkMax(FeederSubsystemConstants.kSparkMotorPortIntakeArm, MotorType.kBrushless);

//   public FeederArmSubsystem() {

//     m_intakeArm.restoreFactoryDefaults();

//     // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
//     // Time to Failure Summary
//     // 20A Limit - Motor survived full 220s test.
//     // 40A Limit - Motor failure at approximately 27s.
//     // 60A Limit - Motor failure at approximately 5.5s
//     // 80A Limit* - Motor failure at approximately 2.0s
//     // Advice: Keep at a 20A limit.

//     m_intakeArm.setSmartCurrentLimit(15);
    
//     // m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
//     // m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

//     // m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, 5);
//     // m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, 5);

//   }

//   public void downFeeder() {
//     System.out.println("FeederSubsystem::downFeeder");
//     m_intakeArm.set(-0.1);  
//   }

//   public void upFeeder() {
//     System.out.println("FeederSubsystem::upFeeder");
//     m_intakeArm.set(0.15);
//   }
  

//   public void stopFeeder() {
//     System.out.println("FeederSubsystem::stopFeeder");
//     m_intakeArm.stopMotor();
//   }
// }
