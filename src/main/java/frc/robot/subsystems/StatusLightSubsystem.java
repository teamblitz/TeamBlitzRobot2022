package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class StatusLightSubsystem extends SubsystemBase{

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue;


    public StatusLightSubsystem() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(24);
        m_led.setLength(m_ledBuffer.getLength());

        // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        //     // Sets the specified LED to the RGB values for red
        //     m_ledBuffer.setRGB(i, 0, 0, 128);
        //  }
         
        // m_led.setData(m_ledBuffer);

        // Set the data

        

        m_led.setData(m_ledBuffer);
        m_led.start();

        rainbow();

        m_led.setData(m_ledBuffer);
        System.out.println("Status Light Subsystem Constructor");

        
    }

    @Override
    public void periodic() {

    }
    
    private void rainbow() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
    
}
