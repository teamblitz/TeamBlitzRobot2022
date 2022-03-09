package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallShooterPlanSubsystem;
import frc.robot.subsystems.LimelightTargetSubsystem;

 public class Target extends CommandBase
{
	DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    BallShooterPlanSubsystem ballShooterPlanSubsystem;
    LimelightTargetSubsystem limelightTargetSubsystem;
    

	
	long	startTime;
    long    targetLastSeen;
	long	notSeenTimeout;
	boolean valid;
	long 	timeout;

	

	public Target(final DriveSubsystem driveSubsystem, final BallShooterPlanSubsystem ballShooterPlanSubsystem, final LimelightTargetSubsystem limelightTargetSubsystem, long notSeenTimeout, long timeout){
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
        this.ballShooterPlanSubsystem = ballShooterPlanSubsystem;
        this.limelightTargetSubsystem = limelightTargetSubsystem;
		this.timeout = timeout;
		this.notSeenTimeout = notSeenTimeout;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Target");
		startTime = System.currentTimeMillis();
        targetLastSeen = System.currentTimeMillis();
		valid = false;

		
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// final long Cur_Time = System.currentTimeMillis();

		
		if (limelightTargetSubsystem.getValid() > 0){ //if limelight sees the ball then this returns true
			valid = true;
			targetLastSeen = System.currentTimeMillis(); //updates ball last seen. as we are seeing it now.
			driveSubsystem.performDrive(0, 0, false, true);
			System.out.println("Target Valid");
		}
		else{
			System.out.printf("No Target, is the limelight obstructed? Ending in %d seconds %n", timeout - (System.currentTimeMillis() - targetLastSeen)); //Prints how long until the command will end due to no ball being seen

		}

		
	}


    // Called when isFinished returns ture
    public void end() {
		driveSubsystem.performDrive(0, 0, false, false);
		System.out.println("Ending Target");
    }

	// Make this return true when this Command no longer needs to run execute()
		// TODO - DOES not work! ASK NOAH.
	@Override
    public boolean isFinished() {
		return (System.currentTimeMillis() - targetLastSeen > notSeenTimeout) /*If we havent seen the target for more than notSeenTimeout, end.*/ || (System.currentTimeMillis() - startTime > timeout); // Ends if either condition is true
	}


}
