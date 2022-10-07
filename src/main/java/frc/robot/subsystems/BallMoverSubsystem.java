/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
  private final CANSparkMax m_ballMoverM = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverR, MotorType.kBrushless);
  // Slave
  private final CANSparkMax m_ballMoverS = new CANSparkMax(BallMoverSubsystemConstants.kSparkMotorPortBallMoverL, MotorType.kBrushless);

  private final StatusManager status = StatusManager.getInstance();

  public BallMoverSubsystem() {
    m_ballMoverM.restoreFactoryDefaults();
    m_ballMoverS.restoreFactoryDefaults();
    // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
    // Time to Failure Summary
    // 20A Limit - Motor survived full 220s test.
    // 40A Limit - Motor failure at approximately 27s.
    // 60A Limit - Motor failure at approximately 5.5s
    // 80A Limit* - Motor failure at approximately 2.0s
    m_ballMoverM.setSmartCurrentLimit(15);
    m_ballMoverS.setSmartCurrentLimit(15);

    m_ballMoverS.follow(m_ballMoverM, true);

    ShuffleboardLayout layout = Shuffleboard.getTab(TelementryConstants.kSubsystemTab).getLayout("Ball Mover", BuiltInLayouts.kGrid);
    layout.addNumber("Left", m_ballMoverS::get);
    layout.addNumber("Right", m_ballMoverM::get);

    status.addMotor(m_ballMoverS, "bllMvrL");
    status.addMotor(m_ballMoverM, "bllMvrR");
  }

  // Enables BallMover Wheels
  public void start() {
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Start");
      }
    set(.5);
  }

  private void set(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    m_ballMoverM.set(speed);
    status.logRevError(m_ballMoverM);
  }

  // Sets the ball mover for intake
  public void startIntake() {
    set(.5);
  }

  // This could reverse BallMover Wheels
  public void reverse(){
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Reverse");
      }
    set(-.5);

   }

  // Disable BallMover Wheels
  public void stop() {
    if (Robot.isSimulation()) {
      System.out.println("Ball Mover Stop");
      }
    set(0);

  }
}
