package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class StatusLightSubsystem extends SubsystemBase{

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    // Constants
    private final long kNumberOfLEDs = 22;
    private final double kHalfWidth = ((double)kNumberOfLEDs - 1.0)/2.0;
    private final int kBrightness = 255;

    private long center;
    private long start;
    private long howMany;
    private int red;
    private int green;
    private int blue;

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

        m_led.start();
        clear();

        // rainbow();

        // m_led.setData(m_ledBuffer);
        System.out.println("Status Light Subsystem Constructor");

    }

    @Override
    public void periodic() {
        // setStatusLights(0, .5, 0);
    }
    

    // private void rainbow() {
        
    //     // For every pixel
    //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //       // Calculate the hue - hue is easier for rainbows because the color
    //       // shape is a circle so only one value needs to precess
    //       final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
    //       // Set the value
    //       m_ledBuffer.setHSV(i, hue, 255, 128);
    //     }
    //     // Increase by to make the rainbow "move"
    //     m_rainbowFirstPixelHue += 3;
    //     // Check bounds
    //     m_rainbowFirstPixelHue %= 180;
    // }
    


    public void setStatusLights(double x, double size, int color) {
        howMany = Math.round(size * 22);
        center = Math.round((x+1) * kHalfWidth);
        start = center - (howMany / 2);

        if (color == 0) { // If we are blue
            red = 0;
            green = 0;
            blue = kBrightness;
        }
        else if (color == 1) { // If we are red
            red = kBrightness;
            green = 0;
            blue = 0;
        }
        else if (color == 2) {// If we are targeting lights will be purple
            red = kBrightness / 2;
            green = 0;
            blue = kBrightness / 2;
        }
        // x value will be our center
        // 11 and 12 are our center.

        // Clear the leds just in case.
        for (int i = 0; i < kNumberOfLEDs; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        
        // Set our leds
        for (int i = (int)start; i < start+howMany; i++) {
            if (i >= 0 && i < kNumberOfLEDs){
                m_ledBuffer.setRGB(i, red, green, blue);
            }
        }
        m_led.setData(m_ledBuffer);
    }
    public void clear () {
        for (int i = 0; i < kNumberOfLEDs; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }
    
}
