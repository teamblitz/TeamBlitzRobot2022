package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax m_shooter = new CANSparkMax(ShooterSubsystemConstants.kSparkMotorPortShooter, MotorType.kBrushless);

  public ShooterSubsystem() {

    m_shooter.restoreFactoryDefaults();
    // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
    // Time to Failure Summary
    // 20A Limit - Motor survived full 220s test.
    // 40A Limit - Motor failure at approximately 27s.
    // 60A Limit - Motor failure at approximately 5.5s
    // 80A Limit* - Motor failure at approximately 2.0s
    // m_shooter.setSmartCurrentLimit(15);  // Do not uncomment this unless you modify the speed below

  }
  // Enables Shooter Wheel
  public void start() {
    if (Robot.isSimulation()) {
    System.out.println("Shooter Start");
    }
    m_shooter.set(0.45);
  }

  // Enables Shooter Wheel
  public void reverse() {
    if (Robot.isSimulation()) {
      System.out.println("Shooter Stop");
    }
    m_shooter.set(-0.31);
  }

    

  // Disable Shooter Wheels
  public void stop() {
    if (Robot.isSimulation()) {
      System.out.println("Shooter Stop");
    }
    m_shooter.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
