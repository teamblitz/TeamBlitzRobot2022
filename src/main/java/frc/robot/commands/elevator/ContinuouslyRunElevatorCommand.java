package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ContinuouslyRunElevatorCommand extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    private boolean goingUp;

    public ContinuouslyRunElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        goingUp = true;
    }

    @Override
    public void execute() {
        if (goingUp) {
            if (elevatorSubsystem.atTop()) {
                goingUp = false;
            } else {
                elevatorSubsystem.upElevator();
            }
        } else {
            if (elevatorSubsystem.atBottom()) {
                goingUp = true;
            } else {
                elevatorSubsystem.downElevator();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
