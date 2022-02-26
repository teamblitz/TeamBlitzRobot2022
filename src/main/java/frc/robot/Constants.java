/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
        public static final int kRightMasterPort = 4;
        public static final int kRightSlavePort = 3;
        
        public static final int kLeftMasterPort = 5;
        public static final int kLeftSlavePort = 6;
    }
    
    public static final class ElevatorConstants {
        public static final int kMasterPort = 7;
        public static final int kSlavePort = 8;
    }
    
    public static final class OIConstants {
        public static final boolean kUseAuxController = false;
        public static final int kDriveControllerPort = 0;
        
        // Xbox buttons:
        public static final int kOverdriveLeftTriggerAxis = 2;
        public static final int kOverdriveRightTriggerAxis = 3;
        
        public static final int kUpElevator = 4; // Should be the Y button
        public static final int kDownElevator = 1; // Should be the A button
        
        public static final int kIntake = 2; // Should be the B button
        public static final int kBallMover = 3; // Should be the X button
        public static final int kShooter = 6;  // Should be right bumper  
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