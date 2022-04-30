package frc.robot.subsystems;


import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Utils;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.*;

public class ElevatorSubsystemTest {
    private ElevatorSubsystem elevatorSubsystem;
    DigitalInput topLimit = mock(DigitalInput.class);
    DigitalInput bottomLimit = mock(DigitalInput.class);
    WPI_TalonFX master;
    WPI_TalonFX slave;
    TalonFXSimCollection masterSim;
    TalonFXSimCollection slaveSim;
    PWMSim h;

    
    @Before
    public void setup() {
        WPIUtilJNI.setMockTime(0); //Set mock time so we can simulate filters without waiting
        WPIUtilJNI.enableMockTime();
        master = new WPI_TalonFX(ElevatorConstants.kMasterPort); // Master
        slave = new WPI_TalonFX(ElevatorConstants.kSlavePort); // Slave
        // master = mock(WPI_TalonFX.class); // Master
        // slave = mock(WPI_TalonFX.class); // Slave
        // masterSim = master.getSimCollection();
        // slaveSim = slave.getSimCollection();
        
        when(topLimit.get()).thenReturn(false); // Disable limit switches
        when(bottomLimit.get()).thenReturn(false);

        elevatorSubsystem = new ElevatorSubsystem( // Call our other with these hardwere inputs
            master, // Master
            slave, // Slave
            topLimit, // Mock top limit
            bottomLimit // Mock bottom limit
        );
        
    }
    @After
    public void cleanup() {
        elevatorSubsystem.close();
        elevatorSubsystem = null;
    }
    // DOES NOT WORK BECAUSE CLASS VERRY DUMB
    // @Test
    // public void checkThatUpMakesElevatorGoUp() {
    //     // when(topLimit.get()).thenReturn(false);
    //     // elevatorSubsystem.test();
    //     // elevatorSubsystem.periodic();
    //     master.setInverted(true);

    //     master.set(.5);
    //     System.out.println(master.get());
    //     // assertTrue(master.getMotorOutputVoltage() != 0.0);
    // }
    // @Test
    // public void checkThatTopLimitSwitchStopsElevatorGoingUp() {
    //     elevatorSubsystem.upElevator();
    // }
}
