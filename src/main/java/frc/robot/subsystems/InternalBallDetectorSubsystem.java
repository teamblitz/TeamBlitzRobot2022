package frc.robot.subsystems;

import java.lang.Math;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.time.StopWatch;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;


public class InternalBallDetectorSubsystem extends SubsystemBase {
    
    // Constants
    private final Color kRedTarget = new Color(.52, .34, .13);
    private final Color kBlueTarget = new Color(.15, .38, .46);
    
    // Do we see red or blue at all?
    private boolean isRed = false;
    private boolean isBlue = false;

    private StopWatch timer = new StopWatch();
     /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */

    private final ColorMatch m_colorMatcher = new ColorMatch();


    public InternalBallDetectorSubsystem() {
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kBlueTarget);
        timer.start();

    }
    
    public boolean ballSeen() {return isRed || isBlue;}
    public int lastSeen() {return timer.getDurationMs();}
    
    
    // Used for debuging only
    private static double CalculateDistance(Color color1, Color color2) {
        double redDiff = color1.red - color2.red;
        double greenDiff = color1.green - color2.green;
        double blueDiff = color1.blue - color2.blue;
    
        return Math.sqrt((redDiff * redDiff + greenDiff * greenDiff + blueDiff * blueDiff) / 2);
      }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Color detectedColor = m_colorSensor.getColor();
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Distance From Red", CalculateDistance(detectedColor, kRedTarget));
        SmartDashboard.putNumber("Distance From Blue", CalculateDistance(detectedColor, kBlueTarget));


        ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
        if (match != null) {
            isRed = (match.color == kRedTarget);
            isBlue = (match.color == kBlueTarget);
        }
        else {
            isRed = false;
            isBlue = false;
        }
        if (isRed || isBlue) {
            timer.start();
        }
        
        SmartDashboard.putString("Is Red?", isRed ? "Yes" : "No");
        SmartDashboard.putString("Is Blue?" , isBlue ? "Yes" : "No");


    }

}
