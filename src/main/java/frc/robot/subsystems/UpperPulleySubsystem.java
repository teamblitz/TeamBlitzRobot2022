/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.UpperPulleySubsystemConstants;

// public class UpperPulleySubsystem extends SubsystemBase {
//   /**
//    * Creates a new UpperPulleySubsystem.
//    */
//   private final TalonSRX m_upperPulleyMotor = new TalonSRX(UpperPulleySubsystemConstants.kUpperPulleyPort);

//   public UpperPulleySubsystem() {

//     //Default Configs
//     m_upperPulleyMotor.configFactoryDefault();
//     m_upperPulleyMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

//     //Config nominal outputs
//     m_upperPulleyMotor.configNominalOutputForward(0, 10);
//     m_upperPulleyMotor.configNominalOutputReverse(0, 10);
//     m_upperPulleyMotor.configPeakOutputForward(1, 10);
//     m_upperPulleyMotor.configPeakOutputReverse(-1, 10);

//     //Config neutral mode
//     m_upperPulleyMotor.setNeutralMode(NeutralMode.Brake);

//     //Config Velocity
//     m_upperPulleyMotor.config_kF(0, 1023.0/7200.0, 10);
//     m_upperPulleyMotor.config_kP(0, 0.25, 10);
//     m_upperPulleyMotor.config_kI(0, 0.001, 10);
//     m_upperPulleyMotor.config_kD(0, 20, 10);

//     m_upperPulleyMotor.configPulseWidthPeriod_EdgesPerRot(20, 10);
//   }

//   //Change the decimal at the end of ControlMode.PercentOutput to determine speed. 
  
//   public void upPulley() {
//     System.out.println("UpperPulleySubsystem::Up");
//     m_upperPulleyMotor.set(ControlMode.PercentOutput, 0.45);
//   }

//   public void downPulley() {
//     System.out.println("UpperPulleySubsystem::Down");
//     m_upperPulleyMotor.set(ControlMode.PercentOutput, -0.45);
//   }

//   public void stopPulley() {
//     System.out.println("UpperPulleySubsystem::Stop");
//     m_upperPulleyMotor.set(ControlMode.PercentOutput, 0.0);
//   }
// }
