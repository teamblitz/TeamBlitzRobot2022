/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
* The Constants class provides a convenient place for teams to hold robot-wide
* numerical or boolean constants. This class should not be used for any other
* purpose. All constants should be declared globally (i.e. public static). Do
* not put anything functional in this class.
*
* <p>
* It is advised to statically import this class (or one of its inner classes)
* wherever the constants are needed, to reduce verbosity.
*/
public final class Constants {
    public static final class DriveConstants {
        public static final int kRightMasterPort = 6;
        public static final int kRightSlavePort = 5;
        
        public static final int kLeftMasterPort = 4;
        public static final int kLeftSlavePort = 3;
    }
    
    public static final class ElevatorConstants {
        public static final int kMasterPort = 8;
        public static final int kSlavePort = 7;
    }
    
    public static final class OIConstants {
        public static final boolean kUseAuxController = false;
        public static final int kDriveControllerPort = Axis.kRightTrigger.value; // Right trigger
        
        // Xbox buttons:
        
        // We can do Button.kA.value to get the value of button A.
        // We could make the constants enums and do .value when binding
        public static final int kOverdriveRightTriggerAxis = 3;
        
        public static final int kUpElevator = Button.kY.value; // Should be the Y button
        public static final int kDownElevator = Button.kA.value; // Should be the A button
        
        public static final int kIntake = Button.kB.value; // Should be the B button
        public static final int kBallMover = Button.kX.value; // Should be the X button
        public static final int kBallMoverReversed = Button.kBack.value; // Back Button
        public static final int kShooter = Button.kRightBumper.value;  // Should be right bumper  
        public static final int kShooterReversed = Button.kStart.value; // Start button
        
        //not actually used but listed here for reference. Actual binding is done in RobotContainer beginTeleop()
        public static final int kSemiAutoBallSeek = Button.kLeftBumper.value; // Auto Ball seek is on left bumper
        public static final int kSemiAutoBallTarget = Axis.kLeftTrigger.value; // Auto target is on left analog trigger

    }
    
    public static final class BallMoverSubsystemConstants {
        public static final int kSparkMotorPortBallMoverR = 9;
        public static final int kSparkMotorPortBallMoverL = 10;
    }

    public static final class IntakeSubsystemConstants {
        public static final int kSparkMotorPortIntake = 11;
    }

    public static final class ShooterSubsystemConstants {
        public static final int kSparkMotorPortShooter = 12;
    }
}