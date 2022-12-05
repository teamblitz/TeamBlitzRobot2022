/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Robot;
import frc.robot.StatusManager;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

    private final CANSparkMax intakeMotor;

    private StatusManager status = StatusManager.getInstance();

    public IntakeSubsystem(CANSparkMax intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.restoreFactoryDefaults();
        status.logRevError(this.intakeMotor);

        // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
        // Time to Failure Summary
        // 20A Limit - Motor survived full 220s test.
        // 40A Limit - Motor failure at approximately 27s.
        // 60A Limit - Motor failure at approximately 5.5s
        // 80A Limit* - Motor failure at approximately 2.0s
        // Advice: Keep at a 20A limit.

        this.intakeMotor.setSmartCurrentLimit(15);
        status.logRevError(this.intakeMotor);

        // Start automatic updating of this motors speed
        Shuffleboard.getTab(TelemetryConstants.SUBSYSTEM_TAB)
                .addNumber("Intake", this.intakeMotor::get);

        status.addMotor(this.intakeMotor, "Intake");
    }

    public IntakeSubsystem() {
        this(new CANSparkMax(IntakeConstants.PORT, MotorType.kBrushless));
    }

    CANSparkMax getIntakeMotor() {
        return intakeMotor;
    }

    public void start() {
        if (Robot.isSimulation()) {
            System.out.println("Intake Start");
        }
        intakeMotor.set(1);
        status.logRevError(intakeMotor);
    }

    public void stop() {
        if (Robot.isSimulation()) {
            System.out.println("Intake Stop");
        }
        intakeMotor.stopMotor();
        status.logRevError(intakeMotor);
    }

    public void reverse() {
        if (Robot.isSimulation()) {
            System.out.println("Intake Reverse");
        }
        intakeMotor.set(-1);
        status.logRevError(intakeMotor);
    }

    public void close() {
        intakeMotor.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }
}
