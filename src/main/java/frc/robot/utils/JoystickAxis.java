package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Essentaly joystick button but for analog imputs
 */
public class JoystickAxis extends Button {
  private final GenericHID m_joystick;
  private final int m_axisNumber;
  private final double m_threshold;

  /**
   * Creates a joystick trigger for triggering commands from xbox axis.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   * @param thresholh The threashold to compair the trigger agents
   */
  // If a threshold is not provided
  public JoystickAxis(GenericHID joystick, int buttonNumber) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    m_axisNumber = buttonNumber;
    m_threshold = 0.5;
  }
  public JoystickAxis(GenericHID joystick, int buttonNumber, double threshold) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    m_axisNumber = buttonNumber;
    m_threshold = threshold;
  }


  /**
   * Gets the value of the joystick axis.
   *
   * @return The value of the joystick axis
   */
  @Override
  public boolean get() {
    return m_joystick.getRawAxis(m_axisNumber) > m_threshold;
  }
}
