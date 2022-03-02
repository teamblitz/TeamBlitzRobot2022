package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallAcquirePlanSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

 public class SeekBall extends CommandBase
{
	DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    BallAcquirePlanSubsystem ballAcquirePlanSubsystem;
    LimelightSubsystem limelightSubsystem;
    

	long	duration;
	long	delay;
	long	startTime;
    long    ballLastSeen;
	double	voltage;
	

	public SeekBall(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, final BallAcquirePlanSubsystem ballAcquirePlanSubsystem, final LimelightSubsystem limelightSubsystem){
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.intakeSubsystem = intakeSubsystem;
        this.ballAcquirePlanSubsystem = ballAcquirePlanSubsystem;
        this.limelightSubsystem = limelightSubsystem;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = System.currentTimeMillis();
        ballLastSeen = System.currentTimeMillis();
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
        if (limelightSubsystem.getValid() > 0){
            ballLastSeen = System.currentTimeMillis();
        }



		final long Cur_Time = System.currentTimeMillis();

        if (Cur_Time - startTime > delay) {
            driveSubsystem.tankDrive(voltage, -voltage);
        }
		
	}


    // Called when isFinished returns ture
    public void end() {
        driveSubsystem.tankDrive(0, 0);
        System.out.println("Ending DriveStraitWith Delay");
    }

	// Make this return true when this Command no longer needs to run execute()
	
	@Override
    public boolean isFinished() {
		return (System.currentTimeMillis() - ballLastSeen) > (duration + delay);
	}


}
