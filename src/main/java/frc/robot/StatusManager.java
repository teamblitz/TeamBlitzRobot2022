package frc.robot;


import java.io.Serializable;
import java.lang.invoke.SerializedLambda;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Keeps track of motor status and convays errors to the dash board
 */
public class StatusManager implements Runnable {
    private static StatusManager instance;

    private final int errorCooldownMs = 2000;

    private final Map<Integer, String> canMotorStatus = new HashMap<>();
    private final Map<Integer, Long> lastError = new HashMap<>();
    private final Set<BaseMotorController> ctreMotors = new HashSet<>();
    private final Set<CANSparkMax> sparkMotors = new HashSet<>();

    private StatusManager() {}

    public static StatusManager getInstance() {
        if (instance == null) {
            instance = new StatusManager();
        }
        return instance;
    }

    /**
     * Should Run Periodicaly 5 times a second
     */
    @Override
    public void run() {
        updateNT();
    }

    public void logRevError(CANSparkMax motor) {
        logRevError(motor.getLastError(), motor.getDeviceId());
    }

    public synchronized void logRevError(REVLibError error, int id) {
        canMotorStatus.put(id, error.toString());
        if (error == REVLibError.kOk) return;
        if (lastError.get(id) == null 
            || lastError.get(id) + errorCooldownMs < System.currentTimeMillis() 
        ) {
            DataLogManager.log("Error: " + error.toString() + " on Spark + " + id);
            lastError.put(id, System.currentTimeMillis());
        }
    }
    
    public void logCTREError(BaseMotorController motor) {
        logCTREError(motor.getLastError(), motor.getDeviceID());
    }

    public synchronized void logCTREError(ErrorCode errorCode, int deviceId) {
        canMotorStatus.put(deviceId, errorCode.toString());
        if (errorCode == ErrorCode.OK) return;
        if (lastError.get(deviceId) == null 
            || lastError.get(deviceId) + errorCooldownMs < System.currentTimeMillis() 
        ) {
            DataLogManager.log("Error: " + errorCode.toString() + " on CTRE Motor + " + deviceId);
            lastError.put(deviceId, System.currentTimeMillis());
        }
    }

    public void addCTRE(BaseMotorController motor) {
        ctreMotors.add(motor);
    }
    public void addSpark(CANSparkMax motor) {
        sparkMotors.add(motor);
    }


    NetworkTable table = NetworkTableInstance.getDefault().getTable("/dashboard/motors");
    private void updateNT() {
        canMotorStatus.forEach((k,v) -> {
            table.getEntry(k.toString()).setString(v); // move the map to nt
        });
    }
}
