/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeSubsystemConstants;;

/**
 * Add your docs here.
  */
public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeSubsystemConstants.kSparkMotorPortIntake, MotorType.kBrushless);

  public IntakeSubsystem() {

    m_intakeMotor.restoreFactoryDefaults();

    // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
    // Time to Failure Summary
    // 20A Limit - Motor survived full 220s test.
    // 40A Limit - Motor failure at approximately 27s.
    // 60A Limit - Motor failure at approximately 5.5s
    // 80A Limit* - Motor failure at approximately 2.0s
    // Advice: Keep at a 20A limit.

    m_intakeMotor.setSmartCurrentLimit(15);
    
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, 5);
    // m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, 5);

  }


  public void start() {
    if (Robot.isSimulation()) {
      System.out.println("Intake Start");
    }
    m_intakeMotor.set(1.0);
  }

  public void stop() {
    if (Robot.isSimulation()) {
      System.out.println("Intake Stop");
    }
    m_intakeMotor.stopMotor();
  }
}
