/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX; //Changed from SRX to FX -- could result in some errors... also check output of motors

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooterMotorTop = new TalonFX(ShooterConstants.kShooterMotorTopPort);
  private final TalonFX m_shooterMotorBottom = new TalonFX(ShooterConstants.kShooterMotorBottomPort);

  private ShuffleboardTab speedcontrols = Shuffleboard.getTab("Controls");

  private NetworkTableEntry topMotorVelocity = Shuffleboard.getTab("Controls")
  .add("Top Motor", m_shooterMotorTop.getSelectedSensorVelocity())
	.withWidget(BuiltInWidgets.kTextView)
	.withPosition(0, 0)
	.withSize(2, 1)
  .getEntry();  
  
  private NetworkTableEntry bottomMotorVelocity = Shuffleboard.getTab("Controls")
  .add("Bottom Motor", m_shooterMotorBottom.getSelectedSensorVelocity())
	.withWidget(BuiltInWidgets.kTextView)
	.withPosition(2, 0)
	.withSize(2, 1)
	.getEntry();

	private double topMotorSpeed;
	private double bottomMotorSpeed;

	/*
	NetworkTableEntry myBoolean = Shuffleboard.getTab("Example Tab")
	.getLayout("List", "Example List")
	.add("My Boolean", false)
	.withWidget("Toggle Button")
	.getEntry();

	NetworkTableEntry mySpeed = Shuffleboard.getTab("Example Tab")
	.add("Bottom Motor", m_shooterMotorBottom.getSelectedSensorVelocity())
	.withWidget(BuiltInWidgets.kSpeedController)
	.getEntry();
	*/

	// ShuffleboardLayout setVelocityCommands = Shuffleboard.getTab("Commands")
  	// .getLayout("Elevator", BuiltInLayouts.kList)
	// .withSize(2, 2);	

	//setVelocityCommands.add("RedZone", m_shooterMotorTop.set(ControlMode.Velocity, 750))

	/*public static void redZone() {
		
	}
	*/
//elevatorCommands.add("PAIN", redZone());
//elevatorCommands.add(new ElevatorUpCommand());

  public ShooterSubsystem() {
	m_shooterMotorTop.configFactoryDefault();
	m_shooterMotorTop.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

	m_shooterMotorBottom.configFactoryDefault();
	m_shooterMotorBottom.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

	// Configure nominal outputs.
	m_shooterMotorTop.configNominalOutputForward(0, 10);
	m_shooterMotorTop.configNominalOutputReverse(0, 10);
	m_shooterMotorTop.configPeakOutputForward(1, 10);
	m_shooterMotorTop.configPeakOutputReverse(-1, 10);

	m_shooterMotorBottom.configNominalOutputForward(0, 50);
	m_shooterMotorBottom.configNominalOutputReverse(0, 25);
	m_shooterMotorBottom.configPeakOutputForward(1, 10);
	m_shooterMotorBottom.configPeakOutputReverse(-1, 10);
	m_shooterMotorBottom.setSensorPhase(true);

	// Configure neutral mode.
	m_shooterMotorTop.setNeutralMode(NeutralMode.Brake);
	m_shooterMotorBottom.setNeutralMode(NeutralMode.Brake);


	// Configure Velocity closed loop gains in slot1.
/*m_shooterMotorTop.config_kF(0, 1023.0/7200.0, 10);
	m_shooterMotorTop.config_kP(0, 0.25, 10);
	m_shooterMotorTop.config_kI(0, 0.001, 10);
	m_shooterMotorTop.config_kD(0, 20, 10);

	m_shooterMotorBottom.config_kF(0, 1023.0/7200.0, 10);
	m_shooterMotorBottom.config_kP(0, 0.25, 10);
	m_shooterMotorBottom.config_kI(0, 0.001, 10);
	m_shooterMotorBottom.config_kD(0, 20, 10);
*/
	m_shooterMotorTop.config_kF(0, 0.0, 10);
	m_shooterMotorTop.config_kP(0, 1, 10);
	m_shooterMotorTop.config_kI(0, 0, 10);
	m_shooterMotorTop.config_kD(0, 0 , 10);

	m_shooterMotorBottom.config_kF(0, 0.0, 10);
	m_shooterMotorBottom.config_kP(0, 1, 10);
	m_shooterMotorBottom.config_kI(0, 0.0, 10);
	m_shooterMotorBottom.config_kD(0, 0, 10);

	//m_shooterMotorTop.configPulseWidthPeriod_EdgesPerRot(20, 10);
	//m_shooterMotorBottom.configPulseWidthPeriod_EdgesPerRot(20, 10);

	m_shooterMotorTop.configPulseWidthPeriod_EdgesPerRot(1024, 10);
	m_shooterMotorBottom.configPulseWidthPeriod_EdgesPerRot(1024, 10);

	m_shooterMotorTop.set(ControlMode.Velocity, 0);
	m_shooterMotorBottom.set(ControlMode.Velocity, 0);
  }

  public void shoot() {
	System.out.println("ShooterSubsytem::shoot");

	//m_shooterMotorTop.set(ControlMode.Velocity, 750);
	//m_shooterMotorBottom.set(ControlMode.Velocity, 750);

// 	m_shooterMotorTop.set(ControlMode.Velocity, 1.0 * topMotorVelocity.getDouble(1.0));
// 	m_shooterMotorBottom.set(ControlMode.Velocity, 1.0 * bottomMotorVelocity.getDouble(1.0));

	m_shooterMotorTop.set(ControlMode.Velocity, topMotorSpeed);
	m_shooterMotorBottom.set(ControlMode.Velocity, bottomMotorSpeed);
	}

  public void stopshooter() {
	System.out.println("ShooterSubsystem::stop");

	m_shooterMotorTop.set(ControlMode.Velocity, 0);
	m_shooterMotorBottom.set(ControlMode.Velocity, 0);

}

public void redZone() {
	topMotorSpeed = 550.0;
	bottomMotorSpeed = 550.0;
	SmartDashboard.putNumber("Top Motor Velocity", topMotorSpeed);
	SmartDashboard.putNumber("Bottom Motor Velocity", bottomMotorSpeed);
}

public void blueZone() {
	topMotorSpeed = 565.0;	
	bottomMotorSpeed = 565.0;
	SmartDashboard.putNumber("Top Motor Velocity", topMotorSpeed);
	SmartDashboard.putNumber("Bottom Motor Velocity", bottomMotorSpeed);
}

public void yellowZone() {
	topMotorSpeed = 250.0;
	bottomMotorSpeed = 850.0;
	SmartDashboard.putNumber("Top Motor Velocity", topMotorSpeed);
	SmartDashboard.putNumber("Bottom Motor Velocity", bottomMotorSpeed);
}

public void greenZone() {
	topMotorSpeed = 150.0;
	bottomMotorSpeed = 1200.0;
	SmartDashboard.putNumber("Top Motor Velocity", topMotorSpeed);
	SmartDashboard.putNumber("Bottom Motor Velocity", bottomMotorSpeed);
}

public void shuffleboardZone() {
	topMotorSpeed = topMotorVelocity.getDouble(1.0);
	bottomMotorSpeed = bottomMotorVelocity.getDouble(1.0);
	SmartDashboard.putNumber("Top Motor Velocity", topMotorSpeed);
	SmartDashboard.putNumber("Bottom Motor Velocity", bottomMotorSpeed);
}
}

	