package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;


public class ButtonBinder extends Button {
  enum InputType {
    XBOX_BUTTON, XBOX_AXIS;
  }

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
  */
  public ButtonBinder(GenericHID joystick, int buttonNumber) {
    super(
      () -> joystick.getRawButton(buttonNumber)
    );
    requireNonNullParam(joystick, "joystick", "ButtonBinder");
  }

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   * @param inputType Accepts a {@link InputType} for telling the object input type
  */
  public ButtonBinder(GenericHID joystick, int buttonNumber, InputType inputType) {
    super(
      inputType == InputType.XBOX_AXIS ? () -> joystick.getRawAxis(buttonNumber) > 0.5 : () -> joystick.getRawButton(buttonNumber)
    );
    requireNonNullParam(joystick, "joystick", "ButtonBinder");
  }

  /**
   * Creates a joystick button for triggering commands.
   * For Xbox Buttons
   *
   * @param joystick The XboxController.
   * @param button XboxController Button enum{@link XboxController.Button}.
  */
  public ButtonBinder(XboxController joystick, XboxController.Button button) {
    super(
      () -> joystick.getRawButton(button.value)
    );
    requireNonNullParam(joystick, "joystick", "ButtonBinder");
  }

  /**
   * Creates a joystick button for triggering commands.
   * For Xbox Triggers
   * 
   * Compairs the value of the axis/trigger to 0.5. If above the command is triggered
   *
   * @param joystick The XboxController.
   * @param axis XboxController Axis enum{@link XboxController.Axis}.
  */
  public ButtonBinder(XboxController joystick, XboxController.Axis axis) {
    super(
      () -> joystick.getRawAxis(axis.value) > 0.5
    );
    requireNonNullParam(joystick, "joystick", "ButtonBinder");
  }
}