package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

 public class DriveStraightWithDelay extends CommandBase
{
	DriveSubsystem driveSubsystem;
	long	duration;
	long	delay;
	long	startTime;
	double	voltage;
	// double	voltageAccommodater	= .5;	// Because our robot is part of the alt-right (or at least leans to the right)
										// Tests: VA Value | Distance | Avg. Deviation | Trials (Deviation to the Right,
										// Negative is Left)
										// 50% 30' 6"/30'; 1"/~5' 10", 12", -1", -10", 19"

	public DriveStraightWithDelay(final DriveSubsystem driveSubsystem, final long duration, final double voltage, final long delay)
	{
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.delay = delay;
		this.voltage = voltage;
		this.duration = duration;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = System.currentTimeMillis();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		 final long Cur_Time = System.currentTimeMillis();
		if (Cur_Time - startTime < delay) {
			driveSubsystem.tankDrive(voltage, voltage);
		}
		else if ((Cur_Time - startTime < (duration + delay))) {
			driveSubsystem.tankDrive(voltage, voltage);
		} else {
			driveSubsystem.tankDrive(0, 0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() - startTime > (duration + delay);
	}


}
