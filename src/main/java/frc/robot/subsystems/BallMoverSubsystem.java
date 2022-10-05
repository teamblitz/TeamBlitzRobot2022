/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StatusManager;
import frc.robot.Constants.BallMoverSubsystemConstants;
import frc.robot.Constants.TelementryConstants;

// TODO: Doesn't actualy use master and slave. instead set right as a master and have left follow it inverted.
public class BallMoverSubsystem extends SubsystemBase {

  // Master
  private final CANSparkMax m_ballMoverR = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverR, MotorType.kBrushless);
  // Slave
  private final CANSparkMax m_ballMoverL = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverL, MotorType.kBrushless);

  private final StatusManager status = StatusManager.getInstance();

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

    ShuffleboardLayout layout = Shuffleboard.getTab(TelementryConstants.kSubsystemTab).getLayout("Ball Mover", BuiltInLayouts.kGrid);
    layout.addNumber("Left", m_ballMoverL::get);
    layout.addNumber("Right", m_ballMoverR::get);

    status.addMotor(m_ballMoverL, "bllMvrL");
    status.addMotor(m_ballMoverR, "bllMvrR");
  }

  // Enables BallMover Wheels
  public void start() {
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Start");
      }
    m_ballMoverR.set(0.65);
    status.logRevError(m_ballMoverR);
    m_ballMoverL.set(-0.65);
    status.logRevError(m_ballMoverL);

  }

  // This could reverse BallMover Wheels
  public void reverse(){
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Reverse");
      }
    m_ballMoverR.set(-0.45);
    status.logRevError(m_ballMoverR);
    m_ballMoverL.set(0.45);
    status.logRevError(m_ballMoverL);

   }

  // Disable BallMover Wheels
  public void stop() {
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Stop");
      }
    m_ballMoverR.set(0.0);
    status.logRevError(m_ballMoverR);
    m_ballMoverL.set(0.0);
    status.logRevError(m_ballMoverL);

  }
}
