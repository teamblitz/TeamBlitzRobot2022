package frc.robot;


import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Keeps track of motor status and conveys errors to the dashboard
 */
public class StatusManager implements Runnable {
    private static StatusManager instance;

    private final int errorCooldownMs = 2000;

    private final Map<Integer, String> canMotorStatus = new HashMap<>();
    private final Map<Integer, Long> lastError = new HashMap<>();

    private final Set<MotorController> motors = new HashSet<>();

    private final Map<MotorController, NetworkTableEntry> motorStatus = new HashMap<>();

    private final double kSparkDisconectedTemperature = 25.0; // Sparkmaxes give a tempurature of 25 when the controller is disconnected.

    private final String kSparkDisconectedFirmware = "v7.-26.1";

    private StatusManager() {}

    public static StatusManager getInstance() {
        if (instance == null) {
            instance = new StatusManager();
        }
        return instance;
    }

    /**
     * Should Run Periodically 5 times a second
     */
    @Override
    public void run() {
        checkMotors();
    }

    public void logRevError(CANSparkMax motor) {
        logRevError(motor.getLastError(), motor.getDeviceId());
    }

    public void logRevError(REVLibError error, int id) {
        canMotorStatus.put(id, error.toString());
        if (error == REVLibError.kOk) return;
        if (lastError.get(id) == null 
            || lastError.get(id) + errorCooldownMs < System.currentTimeMillis()
        ) {
            System.out.println("Error: " + error.toString() + " on Spark + " + id);
            lastError.put(id, System.currentTimeMillis());
        }
    }
    
    public void logCTREError(BaseMotorController motor) {
        logCTREError(motor.getLastError(), motor.getDeviceID());
    }

    public void logCTREError(ErrorCode errorCode, int deviceId) {
        canMotorStatus.put(deviceId, errorCode.toString());
        if (errorCode == ErrorCode.OK) return;
        if (lastError.get(deviceId) == null 
            || lastError.get(deviceId) + errorCooldownMs < System.currentTimeMillis()
        ) {
            System.out.println("Error: " + errorCode.toString() + " on CTRE Motor + " + deviceId);
            lastError.put(deviceId, System.currentTimeMillis());
        }
    }

    private final ShuffleboardTab tab = Shuffleboard.getTab("Motors");

    public void addMotor(MotorController motor, String name){
        motors.add(motor);
        motorStatus.put(motor, tab.add(name + " status", "none").getEntry());
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("/dashboard/motors");
    
    private void checkMotors() {
        for (MotorController motor : motors) {
            if (motor instanceof CANSparkMax) { // If the motor is a sparkMax
                CANSparkMax sparkMax = (CANSparkMax) motor;
                // Check the sparkMax's status.
                if (sparkMax.getMotorTemperature() == kSparkDisconectedTemperature) {
                    System.out.printf("Spark %d reported temperature 25\u00B0c. This is the reported value if the spark is disconnected.%n", sparkMax.getDeviceId());
                    // motorStatus.get(motor).
                }
                SmartDashboard.putString(sparkMax.getDeviceId() +"", sparkMax.getFirmwareString());
                Collection<? extends String> faults = checkSparkFaults(sparkMax);
                if (faults != null) {
                    Shuffleboard.addEventMarker("Faults on sparkMax: " + sparkMax.getDeviceId(), faults.toString(), EventImportance.kCritical);
                    System.err.printf("Faults on sparkMax %d: %s %n", sparkMax.getDeviceId(), faults.toString());
                    // TODO Allert the dashboard here
                    // uh oh, we have faults (tabnine generated this)
                }
                /* I would check the firmware version here as I think it gives a cirtain value if disconnected, but I don't know
                   what that value is
                 */
                
            } else if (motor instanceof BaseMotorController) { // If the motor is a CTRE motor
                BaseMotorController CTREMotor = (BaseMotorController) motor;
                if (CTREMotor.getTemperature() == 0) {
                    System.out.printf("CTRE motor %d reported temperature 0\u00B0c. This is the reported value if the motor is disconnected.%n", CTREMotor.getDeviceID());
                    // TODO Allert the dashboard here
                }

                SmartDashboard.putString(CTREMotor.getDeviceID() +"", CTREMotor.getFirmwareVersion() + "");

                Faults faults = new Faults();
                CTREMotor.getFaults(faults);
                if (faults.hasAnyFault()) {
                    String faultString = faults.toString();
                    Shuffleboard.addEventMarker("Faults on CTRE Motor: " + CTREMotor.getDeviceID(), faultString, EventImportance.kCritical);
                    System.err.printf("Faults on CTRE Motor %d: %s %n", CTREMotor.getDeviceID(), faultString);
                    // TODO Allert the dashboard here
                }
            }

        }
    }

    private Collection<? extends String> checkSparkFaults(CANSparkMax motor) { // Checks the faults of the sparkMax
        Collection<String> faultList = null; // Faults should rarely occur so there is no need to waste memory heaping empty collections
        for (FaultID fault : FaultID.values()) {
            if (motor.getFault(fault)) {
                if (faultList == null) faultList = new ArrayList<String>();
                faultList.add(fault.toString());
            }
        }
        motor.clearFaults();
        return faultList;
    }
}