package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public TargetCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem, this.visionSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(
                visionSubsystem.ballShooterPlan.getFwd(),
                visionSubsystem.ballShooterPlan.getRot(),
                false);
    }
}
