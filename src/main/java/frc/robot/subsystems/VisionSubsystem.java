package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Condenses all Limelight and Plan Subsystems into one Subsystem to decrease verbosity Should
 * function exactly like the pre-existing subsystems. Outside code using vision must be changed to
 * VisionSubsystem.BallAcquire instead of BallAcquire subsystem. All internal methods and code
 * remain intact
 */
public class VisionSubsystem extends SubsystemBase {
    // The object that these variables reference shouldn't change therefor we use the final keyword.
    // Final doesn't stop the object from changing internally, it just stops the reference variable
    // from changing.
    // This way we don't accidentally change them externally.
    public final LimelightCamera ballLimelight;
    public final LimelightCamera targetLimelight;
    public final BallAcquirePlan ballAcquirePlan;
    public final BallShooterPlan ballShooterPlan;

    private final StatusLightSubsystem statusLightSubsystem;
    private final PowerDistribution powerDistribution;

    public VisionSubsystem(StatusLightSubsystem statusLights, PowerDistribution pd) {

        statusLightSubsystem = statusLights;
        powerDistribution = pd;

        ballLimelight =
                new LimelightCamera(
                        "limelight", 0); // Ideally these values would come from constants
        targetLimelight = new LimelightCamera("limelight-target", 7);

        // We don't need to pass these but keep it this way in case we move these classes.
        ballAcquirePlan = new BallAcquirePlan(ballLimelight, statusLightSubsystem);
        ballShooterPlan = new BallShooterPlan(targetLimelight, statusLightSubsystem);
    }

    @Override
    public void periodic() {
        // Call the nested classes execute method for internal calculation
        ballLimelight.execute();
        targetLimelight.execute();
        ballAcquirePlan.execute();
        ballShooterPlan.execute();
    }

    public void statusLightsOff() {
        statusLightSubsystem.clear();
    }

    public void lightsOn() {
        powerDistribution.setSwitchableChannel(true);
    }

    public void lightsOff() {
        powerDistribution.setSwitchableChannel(false);
    }

    // We could move all of the below to its own file without breaking much
    // <<<>>> TODO - Tidy Up Vision Code.
    // Instantiated for each limelight
    public class LimelightCamera {

        private final NetworkTableEntry tvEntry, txEntry, tyEntry, taEntry;
        private final NetworkTable table; // This must be accessible outside the constructor

        private double tv, tx, ty, ta;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private LimelightCamera(String networkTable, Number pipeline) {
            table = NetworkTableInstance.getDefault().getTable(networkTable);
            setPipeline(pipeline);

            tv = tx = ty = ta = 0.0;

            tvEntry = table.getEntry("tv");
            txEntry = table.getEntry("tx");
            tyEntry = table.getEntry("ty");
            taEntry = table.getEntry("ta");
        }

        public double getValid() {
            return (tv);
        }

        public double getX() {
            return (tx);
        }

        public double getY() {
            return (ty);
        }

        public double getArea() {
            return (ta);
        }

        // returns 0 for blue, 1 for red
        // TODO - <<<>>> Move this out of this class as it has nothing to do with limelight
        public int getAllianceColor() {
            return (NetworkTableInstance.getDefault()
                            .getTable("FMSInfo")
                            .getEntry("IsRedAlliance")
                            .getBoolean(false)
                    ? 1
                    : 0);
        }

        // pipelines selected by below
        public static final int SEEK_BLUE_CONTOUR =
                0; // Keep these static. Idealy they would be in constants
        public static final int SEEK_RED_CONTOUR = 1;
        public static final int SEEK_BLUE_CIRCLE_BLOB = 2;
        public static final int SEEK_RED_CIRCLE_BLOB = 3;
        // set vision navigation pipeline
        public void setPipeline(Number pipelineNum) {
            // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNum);
            table.getEntry("pipeline").setNumber(pipelineNum);
        }

        public void execute() { // We must call this manually
            // read values periodically
            // We only need to do this if we are putting to shuffleboard. else we can do this in the
            // getter methods.
            tv = tvEntry.getDouble(0.0);
            tx = txEntry.getDouble(0.0);
            ty = tyEntry.getDouble(0.0);
            ta = taEntry.getDouble(0.0);

            // post to smart dashboard periodically
            SmartDashboard.putNumber("LimelightValid", tv);
            SmartDashboard.putNumber("LimelightX", tx);
            SmartDashboard.putNumber("LimelightY", ty);
            SmartDashboard.putNumber("LimelightArea", ta);
        }
    }

    // Instantiated only once
    public class BallAcquirePlan {
        // all the below can be tinkered with for tuning
        // Should ideally be in constants.
        private final double autoRotationScaleFactor = 0.3;
        private final double autoMoveScaleFactor = 0.9;
        private final double kP = -0.1f;
        private final double min_command = 0.05f;
        private final double maxHeadingError = 10.0;
        private final double maxAreaFraction = 25.0;
        private final double maxDriveSpeedFraction =
                0.30; // how fast we allow the auto drive code to dictate we want to go

        private final LimelightCamera limelight;
        // We do have this instantiated in the outer class. This will over-ride that instantation
        // for within this class,
        // Keep this here for compatibility reasons in case we move the class.
        private final StatusLightSubsystem statusLightSubsystem;

        // these are the calculated movement directives for auto drive
        private double fwd = 0;
        private double rot = 0;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private BallAcquirePlan(LimelightCamera limelight, StatusLightSubsystem statusLights) {
            this.limelight = limelight;
            statusLightSubsystem = statusLights;
        }

        public double getFwd() {
            return (fwd);
        }

        public double getRot() {
            return (rot);
        }

        public void execute() { // We must call this ourselves.

            // inform LimeLight of our alliance color
            // configure alliance color (0=blue, 1=red)
            // this refers to files like RoundBlue and RoundRed in the limelight_configuration
            // repository
            limelight.setPipeline(
                    limelight.getAllianceColor() == 1
                            ? LimelightCamera.SEEK_RED_CONTOUR
                            : LimelightCamera.SEEK_BLUE_CONTOUR);

            rot = 0.0;
            fwd = 0.0;
            // acquire tx and area and valid from LimelightSubsystem
            double tx = limelight.getX();
            double area = limelight.getArea();

            // boolean valid = false;

            if (limelight.getValid() > 0) {
                // TODO: Re write this later, most likely we can use wpilib's pid controller for
                // consciousness
                double heading_error =
                        -tx; // there's a negative here that's negated again later in arcadeDrive()

                // clamp tx magnitude for safety
                heading_error = Math.max(-maxHeadingError, heading_error);
                heading_error = Math.min(heading_error, maxHeadingError);

                double steering_adjust = 0.0f;
                if (tx > 1.0) {
                    steering_adjust = kP * heading_error - min_command;
                } else if (tx < 1.0) {
                    steering_adjust = kP * heading_error + min_command;
                }

                // calculate a drive speed fraction based on area -- smaller area = further away =
                // go
                // faster
                double driveSpeedFraction =
                        (maxAreaFraction - area) / maxAreaFraction; // results in [0.0 ... 1.0]
                // limit the drive speed fraction to m_maxDriveSpeedFraction for safety
                driveSpeedFraction = Math.min(maxDriveSpeedFraction, driveSpeedFraction);

                // negate steering adjust because camera and ball feeder face the rear
                rot = autoRotationScaleFactor * steering_adjust;
                fwd = autoMoveScaleFactor * -driveSpeedFraction;

                // we could post the debug info to the Shuffleboard if we wanted
                SmartDashboard.putNumber(
                        "AutoMove", (autoRotationScaleFactor * driveSpeedFraction));
            }
        }

        public void statusLights() {
            statusLightSubsystem.setStatusLights(
                    limelight.getX() / 25.0,
                    limelight.getArea() / 30.0,
                    limelight.getAllianceColor());
        }
    }

    // Instantiated only once
    public class BallShooterPlan {
        // all the below can be tinkered with for tuning
        // These should Ideally be in constants.
        private final double autoRotationScaleFactor = 0.3;
        private final double autoMoveScaleFactor = 0.9;
        private final double kP =
                -0.1f; // Why are we using the float indicator here? this variable is a double.
        private final double min_command = 0.05f;
        private final double maxHeadingError = 10.0;
        private final double maxOffsetFraction = 2.0;
        private final double maxDriveSpeedFraction =
                0.30; // how fast we allow the auto drive code to dictate we want to go

        private final LimelightCamera limelight;
        // We do have this instantiated in the outer class. This will over-ride that instantation
        // for within this class,
        // Keep this here for compatibility reasons in case we move the class.
        private final StatusLightSubsystem statusLightSubsystem;

        // these are the calculated movement directives for auto drive
        private double fwd = 0;
        private double rot = 0;

        // Private to ensure this is not instantiated elsewhere. Change this if moved
        private BallShooterPlan(LimelightCamera limelight, StatusLightSubsystem statusLights) {
            this.limelight = limelight;
            statusLightSubsystem = statusLights;
        }

        public double getFwd() {
            return (fwd);
        }

        public double getRot() {
            return (rot);
        }

        public void execute() { // We must call this manualy
            // TODO <<<>>> set correct pipeline number
            // No need to update this periodicly
            // limelight.setPipeline(7);

            rot = 0.0;
            fwd = 0.0;
            // acquire tx and area and valid from LimelightSubsystem
            double tx = limelight.getX();
            double ty = limelight.getY();

            // boolean valid = false;

            if (limelight.getValid() > 0) {
                double heading_error =
                        -tx; // there's a negative here that's negated again later in arcadeDrive()

                // clamp tx magnitude for safety
                heading_error = Math.max(-maxHeadingError, heading_error);
                heading_error = Math.min(heading_error, maxHeadingError);

                double steering_adjust = 0.0f;
                if (tx > 1.0) {
                    steering_adjust = kP * heading_error - min_command;
                } else if (tx < 1.0) {
                    steering_adjust = kP * heading_error + min_command;
                }

                // calculate the direction we need to drive to aim the target where we want it
                // if target is below crosshair, then ty will be negative, and otherwise positive.
                // if ty is negative then we are too far back and need to move forward, so direction
                // will be positive 1
                // if ty is positive then we are too close and need to move back so direction will
                // be negative 1
                double driveDirection = (ty < -1 ? 1 : -1);
                // calculate the speed to drive at based on how far off from target we are
                double driveSpeedFraction =
                        0.33 * (Math.abs(ty)) / maxOffsetFraction; // results in [0.0 ... 1.0]
                // limit the drive speed fraction to m_maxDriveSpeedFraction for safety
                driveSpeedFraction = Math.min(maxDriveSpeedFraction, driveSpeedFraction);

                // DON'T negate steering adjust because target camera faces the front
                rot = autoRotationScaleFactor * steering_adjust * 1;
                // -1 below flips the drive direction compared to BallAcquire which faces the rear
                // instead of the front
                fwd = autoMoveScaleFactor * driveSpeedFraction * driveDirection * 1;

                // we could post the debug info to the Shuffleboard if we wanted
                SmartDashboard.putNumber(
                        "AutoTarget", (autoRotationScaleFactor * driveSpeedFraction * -1));
                SmartDashboard.putNumber("GetFwd", (fwd));
            }
        }

        public void statusLights() {
            statusLightSubsystem.setStatusLights(
                    limelight.getX() / 25.0, 1.0 - Math.abs(limelight.getY()) / 20.0, 2);
        }
    }
}
