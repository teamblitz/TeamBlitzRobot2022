package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;


// TODO - <<<>>> Add requires subsystems
public class Target extends CommandBase
{
	DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    VisionSubsystem vision;
    

	
	long	startTime;
    long    targetLastSeen;
	long	notSeenTimeout;
	boolean valid;
	long 	timeout;

	

	public Target(final DriveSubsystem driveSubsystem, final VisionSubsystem vision, long notSeenTimeout, long timeout){
		this.driveSubsystem = driveSubsystem;
		this.vision = vision;
		this.timeout = timeout;
		this.notSeenTimeout = notSeenTimeout;
		
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(driveSubsystem);
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

		
		if (vision.targetLimelight.getValid() > 0){ //if limelight sees the target then this returns true
			valid = true;
			targetLastSeen = System.currentTimeMillis(); //updates ball last seen. as we are seeing it now.
			driveSubsystem.performDrive(0, 0, false, true);
			// System.out.println("Target Valid");
		}
		else{
			System.out.printf("No Target, is the limelight obstructed? Ending in %d miliseconds %n", timeout - (System.currentTimeMillis() - targetLastSeen)); //Prints how long until the command will end due to no target
		}

		
	}


    // Called when isFinished returns ture
    @Override
    public void end(boolean interrupted) {
		driveSubsystem.performDrive(0, 0, false, false);
		System.out.println("Ending Target");
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		// Math.abs(ballShooterPlanSubsystem.getFwd()) < .1 || 
		return (System.currentTimeMillis() - targetLastSeen > notSeenTimeout) || (System.currentTimeMillis() - startTime > timeout); // Ends if either condition is true
	}


}
