package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusLightSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Constants
    private final long numberOfLEDs = 22;
    private final double halfWidth = ((double) numberOfLEDs - 1.0) / 2.0;
    private final int brightness = 255;

    public StatusLightSubsystem() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(24);
        led.setLength(ledBuffer.getLength());

        led.start();
        clear();
    }

    @Override
    public void periodic() {}

    public void setStatusLights(double x, double size, int color) {
        long howMany = Math.round(size * 22);
        long center = Math.round((x + 1) * halfWidth);
        long start = center - (howMany / 2);

        int red;
        int green;
        int blue;

        if (color == 0) { // If we are blue
            red = 0;
            green = 0;
            blue = brightness;
        } else if (color == 1) { // If we are red
            red = brightness;
            green = 0;
            blue = 0;
        } else if (color == 2) { // If we are targeting lights will be purple
            red = brightness / 2;
            green = 0;
            blue = brightness / 2;
        } else {
            red = 0;
            green = 0;
            blue = 0;
        }
        // x value will be our center
        // 11 and 12 are our center.

        // Clear the LEDs just in case.
        for (int i = 0; i < numberOfLEDs; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }

        // Set our LEDs
        for (int i = (int) start; i < start + howMany; i++) {
            if (i >= 0 && i < numberOfLEDs) {
                ledBuffer.setRGB(i, red, green, blue);
            }
        }
        led.setData(ledBuffer);
    }

    public void clear() {
        for (int i = 0; i < numberOfLEDs; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }
}
