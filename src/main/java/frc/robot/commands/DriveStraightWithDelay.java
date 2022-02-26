package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DriveStraightWithDelay extends CommandBase {
    
    DriveSubsystem driveSubsystem;
	long	duration;
	long	delay;
	long	startTime;
	double	voltage;

    ShooterSubsystem shooterSubsystem;
    long    shooter_duration;
    long    shooter_delay;

    int     stage = 1;
    // double	voltageAccommodater	= .5;	// Because our robot is part of the alt-right (or at least leans to the right)
										// Tests: VA Value | Distance | Avg. Deviation | Trials (Deviation to the Right,
										// Negative is Left)
										// 50% 30' 6"/30'; 1"/~5' 10", 12", -1", -10", 19"

    public DriveStraightWithDelay(final DriveSubsystem driveSubsystem, final long duration, final double voltage, final long delay,
    final ShooterSubsystem shooterSubsystem, final long shooter_duration, final long shooter_delay) {
        // Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.delay = delay;
		this.voltage = voltage;
		this.duration = duration;

        this.shooterSubsystem = shooterSubsystem;
        this.shooter_delay = shooter_delay;
        this.shooter_duration = shooter_duration;
    }

    // Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = System.currentTimeMillis();
	}

    // Called repeatedly when this Command is scheduled to run
    @Override
	public void execute() {

        if (stage==3)
        {
            return;
        }

		final long Cur_Time = System.currentTimeMillis();
        
        if (stage==1){
            //Insert shooter code
            if (Cur_Time - startTime < shooter_delay) {
                System.out.print("S1 First condition");
                shooterSubsystem.start();
                driveSubsystem.tankDrive(-voltage, voltage);
            }
            else if ((Cur_Time - startTime < (shooter_duration + shooter_delay))) {
                shooterSubsystem.start();
                driveSubsystem.tankDrive(-voltage, voltage);
                System.out.print("S1 Second condition");  // This makes it work; don't remove; don't ask
            }
            else {
                shooterSubsystem.stop();
                System.out.println("S1 Stop condition. Entering stage 2");
                stage = 2;
            }
        }

        if (stage==2){
            //Insert drive code.
            System.out.println("stage 2");
            if (Cur_Time - startTime - shooter_duration < delay) {
                System.out.print("S2 First condition");
                driveSubsystem.tankDrive(-voltage, voltage);
            }
            else if ((Cur_Time - startTime - shooter_duration < (duration + delay))) {
                driveSubsystem.tankDrive(-voltage, voltage);
                System.out.println("S2 Second condition");  // This makes it work; don't remove; don't ask
            } else {
                driveSubsystem.tankDrive(0, 0);
                System.out.println("S2 Stop condition. Entering Stage 3");
                stage = 3;
            }
        }

        // if (Cur_Time - startTime < shooter_delay) {
        //     // System.out.print("First condition");
		// 	shooterSubsystem.start();
		// }
		// else if ((Cur_Time - startTime < (shooter_duration + shooter_delay))) {
		// 	shooterSubsystem.start();
        //     // System.out.print("Second condition");  // This makes it work; don't remove; don't ask
		// } else {
		// 	shooterSubsystem.stop();
        //     // System.out.print("Stop condition");
        // }

        // if (Cur_Time - startTime < delay) {
        //     System.out.print("First condition");
        //     driveSubsystem.tankDrive(-voltage, voltage);
        // }
        // else if ((Cur_Time - startTime < (duration + delay))) {
        //     driveSubsystem.tankDrive(-voltage, voltage);
        //     System.out.print("Second condition");  // This makes it work; don't remove; don't ask
        // } else {
        //     driveSubsystem.tankDrive(0, 0);
        //     System.out.print("Stop condition");
        }
    // Make this return true when this Command no longer needs to run execute()
	// @Override
	// public boolean isFinished() {
	// 	return System.currentTimeMillis() - startTime > (duration + shooter_duration + delay);
    // }

}