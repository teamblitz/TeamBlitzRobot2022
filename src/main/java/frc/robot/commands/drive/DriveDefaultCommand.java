package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends CommandBase {
    private final DoubleSupplier fwdSupplier;
    private final DoubleSupplier rotSupplier;
    private final DriveSubsystem drive;

    public DriveDefaultCommand(DoubleSupplier fwd, DoubleSupplier rot, DriveSubsystem drive) {
        fwdSupplier = fwd;
        rotSupplier = rot;
        this.drive = drive;
        addRequirements(drive);
        setName("Teleop Drive");
    }

    @Override
    public void execute() {
        drive.arcadeDrive(fwdSupplier.getAsDouble(), rotSupplier.getAsDouble(), true);
    }
}
