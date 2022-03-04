package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMoverSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

 public class Shoot extends CommandBase
{
	ShooterSubsystem shooterSubsystem;
    BallMoverSubsystem ballMoverSubsystem;
	long	duration;
    long    warmupPeriod;
	long	startTime;

	

	public Shoot(final ShooterSubsystem shooterSubsystem, final BallMoverSubsystem ballMoverSubsystem, final long warmupPeriod, final long duration){
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.shooterSubsystem = shooterSubsystem;
		this.warmupPeriod = warmupPeriod;
		this.duration = duration;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Shoot");
		startTime = System.currentTimeMillis();
        ballMoverSubsystem.start();
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		System.out.println("Shoot Main Loop");

		final long Cur_Time = System.currentTimeMillis();
        if (Cur_Time - startTime > warmupPeriod) {
            shooterSubsystem.start();
        }
		
	}


    // Called when isFinished returns ture
    public void end() {
        shooterSubsystem.stop();
        ballMoverSubsystem.stop();
        System.out.println("Ending Shoot");
    }

	// Make this return true when this Command no longer needs to run execute()
	
	@Override
    public boolean isFinished() {
		return System.currentTimeMillis() - startTime > (duration);
	}


}
