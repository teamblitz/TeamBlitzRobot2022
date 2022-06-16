package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StatusManager;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TelementryConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  
  private final CANSparkMax m_shooter = new CANSparkMax(ShooterConstants.kSparkMotorPortShooter, MotorType.kBrushless);
  
  private final StatusManager status = StatusManager.getInstance();

  public ShooterSubsystem() {

    m_shooter.restoreFactoryDefaults();
    status.logRevError(m_shooter);
    // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
    // Time to Failure Summary
    // 20A Limit - Motor survived full 220s test.
    // 40A Limit - Motor failure at approximately 27s.
    // 60A Limit - Motor failure at approximately 5.5s
    // 80A Limit* - Motor failure at approximately 2.0s
    // m_shooter.setSmartCurrentLimit(15);  // Do not uncomment this unless you modify the speed below

    // Start automatic updating of this motors speed
    Shuffleboard.getTab(TelementryConstants.kSubsystemTab).addNumber("Shooter", m_shooter::get);

    status.addSpark(m_shooter);
  }
  // Enables Shooter Wheel
  public void start() {
    if (Robot.isSimulation()) {
    System.out.println("Shooter Start");
    }
    m_shooter.set(ShooterConstants.kSpeed);
    status.logRevError(m_shooter);
  }

  // Enables Shooter Wheel
  public void reverse() {
    if (Robot.isSimulation()) {
      System.out.println("Shooter Stop");
    }
    m_shooter.set(ShooterConstants.kReverseSpeed);
    status.logRevError(m_shooter);
  }

  // Disable Shooter Wheels
  public void stop() {
    if (Robot.isSimulation()) {
      System.out.println("Shooter Stop");
    }
    m_shooter.set(0.0);
    status.logRevError(m_shooter);
  }

  CANSparkMax getShooter() { // Needed for unit testing
    return m_shooter;
  }

  @Override
  public void close() {
    m_shooter.close(); // Disasemble the shooter
    CommandScheduler.getInstance().unregisterSubsystem(this); // De-regester the subsystem
  }
}
