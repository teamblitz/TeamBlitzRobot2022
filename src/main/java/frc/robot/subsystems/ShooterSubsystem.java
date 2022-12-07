package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.StatusManager;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

    private final CANSparkMax shooter =
            new CANSparkMax(ShooterConstants.PORT, MotorType.kBrushless);

    private final RelativeEncoder encoder =
            shooter.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    private final StatusManager statusManager = StatusManager.getInstance();

    private final NetworkTableEntry speedEntry;

    public ShooterSubsystem() {

        shooter.restoreFactoryDefaults();
        statusManager.logRevError(shooter);
        // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
        // Time to Failure Summary
        // 20A Limit - Motor survived full 220s test.
        // 40A Limit - Motor failure at approximately 27s.
        // 60A Limit - Motor failure at approximately 5.5s
        // 80A Limit* - Motor failure at approximately 2.0s
        // shooter.setSmartCurrentLimit(15);  // Do not uncomment this unless you modify the speed
        // below

        // Start automatic updating of this motors speed
        statusManager.addMotor(shooter, "Shoot");

        Shuffleboard.getTab("test").addNumber("Shooter faults", shooter::getFaults);
        Shuffleboard.getTab("Video").addNumber("Shooter rpm", () -> encoder.getVelocity());
        speedEntry =
                Shuffleboard.getTab("Video")
                        .add("Shooter Speed", ShooterConstants.SPEED)
                        .getEntry();
    }
    // Enables Shooter Wheel
    public void start() {
        if (Robot.isSimulation()) {
            System.out.println("Shooter Start");
        }
        shooter.set(speedEntry.getDouble(ShooterConstants.SPEED));
        statusManager.logRevError(shooter);
        SmartDashboard.putNumber("Firmware", shooter.getFirmwareVersion());
    }

    // Enables Shooter Wheel
    public void reverse() {
        if (Robot.isSimulation()) {
            System.out.println("Shooter Stop");
        }
        shooter.set(ShooterConstants.REVERSE_SPEED);
        statusManager.logRevError(shooter);
    }

    // Disable Shooter Wheels
    public void stop() {
        if (Robot.isSimulation()) {
            System.out.println("Shooter Stop");
        }
        shooter.set(0.0);
        statusManager.logRevError(shooter);
    }

    CANSparkMax getShooter() { // Needed for unit testing
        return shooter;
    }

    @Override
    public void close() {
        shooter.close(); // Close the shooter
        CommandScheduler.getInstance().unregisterSubsystem(this); // De-regester the subsystem
    }
}
