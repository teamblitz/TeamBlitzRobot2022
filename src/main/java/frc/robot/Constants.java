/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.SaitekX52Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int RIGHT_MASTER_PORT = 6;
        public static final int RIGHT_SLAVE_PORT = 5;

        public static final int LEFT_MASTER_PORT = 4;
        public static final int LEFT_SLAVE_PORT = 3;
    }

    public static final class ElevatorConstants {
        public static final int MASTER_PORT = 8;
        public static final int SLAVE_PORT = 7;
        public static final int TOP_LIMIT_PORT = 0;
        public static final int BOTTOM_LIMIT_PORT = 1;
        public static final float UP_SPEED = -0.4f;
        public static final float DOWN_SPEED = 0.4f;
    }

    public static final class OIConstants {

        // Choose 1, not both.
        public static final boolean USE_XBOX_CONTROLLER = false;
        public static final boolean USE_SAITEK_CONTROLLER = true;

        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int BUTTON_BOX_PORT = 1;

        // Xbox buttons:

        public static final class XboxMappings {

            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final XboxController.Axis OVERDRIVE = XboxController.Axis.kRightTrigger;

            public static final XboxController.Button UP_ELEVATOR =
                    XboxController.Button.kY; // Should be the Y button
            public static final XboxController.Button DOWN_ELEVATOR =
                    XboxController.Button.kA; // Should be the A button

            public static final XboxController.Button INTAKE =
                    XboxController.Button.kB; // Should be the B button
            public static final XboxController.Button BALL_MOVER =
                    XboxController.Button.kX; // Should be the X button
            public static final XboxController.Button BALL_MOVER_REVERSED =
                    XboxController.Button.kBack; // Back Button
            public static final XboxController.Button SHOOTER =
                    XboxController.Button.kRightBumper; // Should be right bumper
            public static final XboxController.Button SHOOTER_REVERSED =
                    XboxController.Button.kStart; // Start button

            public static final XboxController.Button SEMI_AUTO_BALL_SEEK =
                    XboxController.Button.kLeftBumper; // Auto Ball seek is on left bumper
            public static final XboxController.Axis SEMI_AUTO_BALL_TARGET =
                    XboxController.Axis.kLeftTrigger; // Auto target is on left analog trigger
        }

        public static final class SaitekMappings {
            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final SaitekX52Joystick.Axis kThrotle = SaitekX52Joystick.Axis.kThrotle;

            public static final SaitekX52Joystick.Button UP_ELEVATOR = SaitekX52Joystick.Button.kT1;
            public static final SaitekX52Joystick.Button DOWN_ELEVATOR =
                    SaitekX52Joystick.Button.kT2;

            public static final SaitekX52Joystick.Button INTAKE = SaitekX52Joystick.Button.kFire;
            public static final SaitekX52Joystick.Button INTAKE_REVERSED =
                    SaitekX52Joystick.Button.kHatDown;
            public static final SaitekX52Joystick.Button BALL_MOVER =
                    SaitekX52Joystick.Button.kUpperTrigger1;
            public static final SaitekX52Joystick.Button BALL_MOVER_REVERSED =
                    SaitekX52Joystick.Button.kHatDown;
            public static final SaitekX52Joystick.Button SHOOTER =
                    SaitekX52Joystick.Button.kLowerTrigger;
            public static final SaitekX52Joystick.Button SHOOTER_REVERSED =
                    SaitekX52Joystick.Button.kHatDown;

            public static final SaitekX52Joystick.Button SEMI_AUTO_BALL_SEEK =
                    SaitekX52Joystick.Button.kA;
            public static final SaitekX52Joystick.Button SEMI_AUTO_BALL_TARGET =
                    SaitekX52Joystick.Button.kB;
        }

        public static final class ButtonBoxMappings {
            public static final int UP_ELEVATOR = ButtonBox.Button.kL1.value;
            public static final int DOWN_ELEVATOR = ButtonBox.Button.kL2.value;

            public static final int INTAKE = ButtonBox.Button.kX.value;
            public static final int INTAKE_REVERSED = ButtonBox.Button.kL3.value;
            public static final int BALL_MOVER = ButtonBox.Button.kY.value;
            public static final int BALL_MOVER_REVERSED = ButtonBox.Button.kR3.value;
            public static final int SHOOTER = ButtonBox.Button.kR1.value;
            public static final int SHOOTER_REVERSED = ButtonBox.Button.kB.value;

            public static final int SEMI_AUTO_BALL_SEEK = ButtonBox.Button.kA.value;
            public static final int SEMI_AUTO_BALL_TARGET = ButtonBox.Button.kR2.value;
        }
    }

    public static final class BallMoverConstants {
        public static final int RIGHT_PORT = 9;
        public static final int LEFT_PORT = 10;
    }

    public static final class IntakeConstants {
        public static final int PORT = 11;
    }

    public static final class ShooterConstants {
        public static final int PORT = 12;
        public static final float SPEED = 0.60f;
        public static final float REVERSE_SPEED = -0.3f;
    }

    public static class TelemetryConstants {
        public static final String SUBSYSTEM_TAB = "Subsystems";
    }
}
