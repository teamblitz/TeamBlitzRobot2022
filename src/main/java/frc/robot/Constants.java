/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SaitekX52Joystick;

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
        public static final int kTopLimitPort = 8;
        public static final int kBottomLimitPort = 7;
        public static final float kUpSpeed = -0.4f;
        public static final float kDownSpeed = 0.4f;
    }
    
    public static final class OIConstants {

        // Choose 1, not both.
        public static final boolean useXboxController = false;
        public static final boolean useSaitekController = true;
        
        public static final int kDriveControllerPort = 0; 
        public static final int kButtonBoxPort = 1;
        
        // Xbox buttons:
        
        public static final class XboxMappings {

            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final XboxController.Axis kOverdrive =  XboxController.Axis.kRightTrigger;
            
            public static final XboxController.Button kUpElevator = XboxController.Button.kY; // Should be the Y button
            public static final XboxController.Button kDownElevator = XboxController.Button.kA; // Should be the A button
            
            public static final XboxController.Button kIntake = XboxController.Button.kB; // Should be the B button
            public static final XboxController.Button kBallMover = XboxController.Button.kX; // Should be the X button
            public static final XboxController.Button kBallMoverReversed = XboxController.Button.kBack; // Back Button
            public static final XboxController.Button kShooter = XboxController.Button.kRightBumper;  // Should be right bumper  
            public static final XboxController.Button kShooterReversed = XboxController.Button.kStart; // Start button
            
            public static final XboxController.Button kSemiAutoBallSeek = XboxController.Button.kLeftBumper; // Auto Ball seek is on left bumper
            public static final XboxController.Axis kSemiAutoBallTarget = XboxController.Axis.kLeftTrigger; // Auto target is on left analog trigger
        }

        public static final class SaitekMappings {
            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final SaitekX52Joystick.Axis kThrotle = SaitekX52Joystick.Axis.kThrotle;
            
            public static final SaitekX52Joystick.Button kUpElevator = SaitekX52Joystick.Button.kT1; // Should be the Y button
            public static final SaitekX52Joystick.Button kDownElevator = SaitekX52Joystick.Button.kT2; // Should be the A button
            
            public static final SaitekX52Joystick.Button kIntake = SaitekX52Joystick.Button.kUpperTrigger1; // Should be the B button
            public static final SaitekX52Joystick.Button kBallMover = SaitekX52Joystick.Button.kLowerTrigger; // Should be the X button
            public static final SaitekX52Joystick.Button kBallMoverReversed = SaitekX52Joystick.Button.kI; // Back Button
            public static final SaitekX52Joystick.Button kShooter = SaitekX52Joystick.Button.kFire;  // Should be right bumper  
            public static final SaitekX52Joystick.Button kShooterReversed = SaitekX52Joystick.Button.kI; // Start button
            
            public static final SaitekX52Joystick.Button kSemiAutoBallSeek = SaitekX52Joystick.Button.kA; // Auto Ball seek is on left bumper
            public static final SaitekX52Joystick.Button kSemiAutoBallTarget = SaitekX52Joystick.Button.kB; // Auto target is on left analog trigger
        }

        public static final class ButtonBoxMappings {
            public static final int kUpElevator = 0;
            public static final int kDownElevator = 0;
            
            public static final int kIntake = 0;
            public static final int kBallMover = 0;
            public static final int kBallMoverReversed = 0;
            public static final int kShooter = 0; 
            public static final int kShooterReversed = 0;

            public static final int kSemiAutoBallSeek = 0;
            public static final int kSemiAutoBallTarget = 0;
        }
    }
    
    public static final class BallMoverSubsystemConstants {
        public static final int kSparkMotorPortBallMoverR = 9;
        public static final int kSparkMotorPortBallMoverL = 10;
    }

    public static final class IntakeSubsystemConstants {
        public static final int kSparkMotorPortIntake = 11;
    }

    public static final class ShooterConstants {
        public static final int kSparkMotorPortShooter = 12;
        public static final float kSpeed = 0.45f;
        public static final float kReverseSpeed = -0.3f;
    }
    public static class TelementryConstants {
        public static final String kSubsystemTab = "Subsystems";
    }
}