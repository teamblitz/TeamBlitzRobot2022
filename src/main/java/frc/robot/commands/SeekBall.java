package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallAcquirePlanSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.InternalBallDetectorSubsystem;

 public class SeekBall extends CommandBase
{
	DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    BallAcquirePlanSubsystem ballAcquirePlanSubsystem;
    LimelightSubsystem limelightSubsystem;
	InternalBallDetectorSubsystem internalBallDetectorSubsystem;
    

	
	long	startTime;
    long    ballLastSeen;
	boolean	seenBall;
	boolean valid;
	long notSeenTimeout;
	long timeout;

	

	public SeekBall(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, final BallAcquirePlanSubsystem ballAcquirePlanSubsystem, final LimelightSubsystem limelightSubsystem, long notSeenTimeout, long timeout, InternalBallDetectorSubsystem internalBallDetectorSubsystem){
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.intakeSubsystem = intakeSubsystem;
        this.ballAcquirePlanSubsystem = ballAcquirePlanSubsystem;
        this.limelightSubsystem = limelightSubsystem;
		this.internalBallDetectorSubsystem = internalBallDetectorSubsystem;
		this.notSeenTimeout = notSeenTimeout;
		this.timeout = timeout;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Seek Ball");
		startTime = System.currentTimeMillis();
        ballLastSeen = System.currentTimeMillis();
		seenBall = false;
		valid = false;
		intakeSubsystem.start();
		
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// final long Cur_Time = System.currentTimeMillis();

		
		if (limelightSubsystem.getValid() > 0){ //if limelight sees the ball then this returns true
			valid = true;

			ballLastSeen = System.currentTimeMillis(); //updates ball last seen. as we are seeing it now.
			driveSubsystem.performDrive(0, 0, true, false);
			// System.out.println("SeekBall Valid");
		}
		else{
			System.out.printf("No Ball, Ending in %d miliseconds %n", notSeenTimeout - (System.currentTimeMillis() - ballLastSeen)); //Prints how long until the command will end due to no ball being seen

		}

		
	}


    // Called when isFinished returns ture
	@Override
    public void end(boolean interrupted) {
		intakeSubsystem.stop();
		driveSubsystem.performDrive(0, 0, false, false);
		System.out.println("Ending SeekBall");
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		return internalBallDetectorSubsystem.ballSeen() || ((System.currentTimeMillis() - ballLastSeen) > notSeenTimeout) || (System.currentTimeMillis() - startTime > timeout);
	}


}
