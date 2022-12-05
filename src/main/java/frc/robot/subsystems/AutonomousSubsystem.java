package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomousSubsystem extends SubsystemBase {

    // Have we left the tarmat?
    private boolean hasLeftTarmat;
    // How much time is left?
    private long autoTimeLeftMS;

    // Should we use vision or dead reakoning?
    private boolean useVision;

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            autoPeriodic();
        }
    }

    private void autoPeriodic() {
        autoTimeLeftMS = Math.round(1000 * DriverStation.getMatchTime());
    }
}
