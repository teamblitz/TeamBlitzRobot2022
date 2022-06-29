package frc.robot;


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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

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
            System.out.println("Error: " + error.toString() + " on Spark + " + id);
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
            System.out.println("Error: " + errorCode.toString() + " on CTRE Motor + " + deviceId);
            lastError.put(deviceId, System.currentTimeMillis());
        }
    }

    public void addCTRE(BaseMotorController motor) {
        ctreMotors.add(motor);
        table.getSubTable("/"+motor.getDeviceID()).getEntry("type").setString("CTRE");
    }
    public void addSpark(CANSparkMax motor) {
        sparkMotors.add(motor);
        table.getSubTable("/"+motor.getDeviceId()).getEntry("type").setString("Spark");
    }
    PowerDistribution m_pd = new PowerDistribution(1, ModuleType.kRev);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("/dashboard/motors");
    
    private void updateNT() {
        canMotorStatus.forEach((k,v) -> {
            table.getSubTable("/"+k).getEntry("error").setString(v.charAt(0) == 'k' ? v.substring(1) : v); // move the map to nt
        });
        sparkMotors.forEach((m) -> {
            table.getSubTable("/"+m.getDeviceId()).getEntry("speed").setNumber(m.get());
            table.getSubTable("/"+m.getDeviceId()).getEntry("temp").setNumber(m.getMotorTemperature());
            table.getSubTable("/"+m.getDeviceId()).getEntry("appliedOutput").setNumber(m.getAppliedOutput());
            table.getSubTable("/"+m.getDeviceId()).getEntry("outputCurrent").setNumber(m.getOutputCurrent());
            table.getSubTable("/"+m.getDeviceId()).getEntry("busVoltage").setNumber(m.getBusVoltage());
            table.getSubTable("/"+m.getDeviceId()).getEntry("status").setString(Robot.isSimulation() ? "sim" : 
                                                                                DriverStation.isDisabled() ? "disabled" :
                                                                                m.get() != 0 && m.getAppliedOutput() == 0 ? "noResponse" :
                                                                                m.getMotorTemperature() == 25 ? "temp" :
                                                                                m.getBusVoltage() < 11 ? "BusVolts ": "ok");
        });
        ctreMotors.forEach((m) -> {
            MotorController mc = (MotorController) m;  // If this errors than somthing is wrong. Should work with tallons
            table.getSubTable("/"+m.getDeviceID()).getEntry("speed").setNumber(mc.get());
            table.getSubTable("/"+m.getDeviceID()).getEntry("temp").setNumber(m.getTemperature());
            table.getSubTable("/"+m.getDeviceID()).getEntry("motorOutputPercent").setNumber(m.getMotorOutputPercent());
            table.getSubTable("/"+m.getDeviceID()).getEntry("motorOutputVolts").setNumber(m.getMotorOutputVoltage());
            table.getSubTable("/"+m.getDeviceID()).getEntry("busVoltage").setNumber(m.getBusVoltage());
            table.getSubTable("/"+m.getDeviceID()).getEntry("status").setString(Robot.isSimulation() ? "sim" : 
                                                                                DriverStation.isDisabled() ? "disabled" :
                                                                                mc.get() != 0 && m.getMotorOutputPercent() == 0 || mc.get() != 0 && m.getMotorOutputVoltage() == 0 ? "noResponse" :
                                                                                m.getTemperature() == 0 ? "temp" :
                                                                                m.getBusVoltage() < 11 ? "BusVolts ": "ok");
        });

    }
}
