package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InternalBallDetectorSubsystem;

 public class DriveStraightWithDelay extends CommandBase
{
	DriveSubsystem driveSubsystem;
	InternalBallDetectorSubsystem internalBallDetectorSubsystem;
	long	duration;
	long	delay;
	long	startTime;
	double	voltage;
	
	Boolean driveBack; // Do we need to drive back? if our ball detector was active recently then this won't activate as we got a ball
	

	public DriveStraightWithDelay(final DriveSubsystem driveSubsystem, InternalBallDetectorSubsystem internalBallDetectorSubsystem, final long duration, final double voltage, final long delay)
	{
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.internalBallDetectorSubsystem = internalBallDetectorSubsystem;
		this.delay = delay;
		this.voltage = voltage;
		this.duration = duration;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Drive");
		startTime = System.currentTimeMillis();
		driveBack = internalBallDetectorSubsystem.lastSeen() < 7000; //If we haven't seen the ball via internal detector in 7 seconds we need to drive backwords as our bot didn't pick up the ball
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// System.out.println("Drive Loop");

		final long Cur_Time = System.currentTimeMillis();
        if ((Cur_Time - startTime > delay) && driveBack) {
            driveSubsystem.tankDrive(voltage, -voltage);
        }
		
	}


    // Called when isFinished returns ture
	@Override
    public void end(boolean interrupted) {
        driveSubsystem.tankDrive(0, 0);
        System.out.println("Ending DriveStraitWith Delay");
    }

	// Make this return true when this Command no longer needs to run execute()
	
	@Override
    public boolean isFinished() {
		return System.currentTimeMillis() - startTime > (duration + delay);
	}


}
