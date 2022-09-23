package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;

public class NavSubsystem extends SubsystemBase{

    //public final AHRS gyro;
    private Pose2d currentPos;

    // private final KalmanFilter filter = new KalmanFilter<>(states, outputs, plant, stateStdDevs, measurementStdDevs, dtSeconds)

    //private final Pose2d[] aprilTagHistory; // Shift it down?

    
    
    // public NavSubsystem() {
    //     gyro = new AHRS(SPI.Port.kMXP);


    // }

    @Override
    public void periodic() {

    }

    // void 

}
