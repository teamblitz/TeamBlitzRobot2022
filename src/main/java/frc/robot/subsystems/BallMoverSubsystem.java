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
import frc.robot.Constants.BallMoverConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Robot;
import frc.robot.StatusManager;

public class BallMoverSubsystem extends SubsystemBase {

    // Master
    private final CANSparkMax ballMoverM =
            new CANSparkMax(BallMoverConstants.RIGHT_PORT, MotorType.kBrushless);
    // Slave
    private final CANSparkMax ballMoverS =
            new CANSparkMax(BallMoverConstants.LEFT_PORT, MotorType.kBrushless);

    private final StatusManager statusManager = StatusManager.getInstance();

    public BallMoverSubsystem() {
        ballMoverM.restoreFactoryDefaults();
        ballMoverS.restoreFactoryDefaults();
        // ISSUE: This was set under 20 amps due to locked rotor testing with sparkmaxes
        // Time to Failure Summary
        // 20A Limit - Motor survived full 220s test.
        // 40A Limit - Motor failure at approximately 27s.
        // 60A Limit - Motor failure at approximately 5.5s
        // 80A Limit* - Motor failure at approximately 2.0s
        ballMoverM.setSmartCurrentLimit(15);
        ballMoverS.setSmartCurrentLimit(15);

        ballMoverS.follow(ballMoverM, true);

        ShuffleboardLayout layout =
                Shuffleboard.getTab(TelemetryConstants.SUBSYSTEM_TAB)
                        .getLayout("Ball Mover", BuiltInLayouts.kGrid);
        layout.addNumber("Left", ballMoverS::get);
        layout.addNumber("Right", ballMoverM::get);

        statusManager.addMotor(ballMoverS, "bllMvrL");
        statusManager.addMotor(ballMoverM, "bllMvrR");
    }

    // Enables BallMover Wheels
    public void start() {
        if (Robot.isSimulation()) {
            System.out.println("Ball Mover Start");
        }
        set(.65);
    }

    private void set(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        ballMoverM.set(speed);
        statusManager.logRevError(ballMoverM);
    }

    // Sets the ball mover for intake
    public void startIntake() {
        set(.5);
    }

    public void reverse() {
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
