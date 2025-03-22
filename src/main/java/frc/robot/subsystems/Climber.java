// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class Climber extends SubsystemBase {

	private final TalonFX m_motor;
	private final VoltageOut m_motorRequest = new VoltageOut(0);
	private final CANcoder m_encoder;

	private final Servo m_Servo;

	/** Creates a new ClimberPivot. */
	public Climber(int motorID, int servoChannel) {

		m_motor = new TalonFX(motorID, "Canivore");
		m_encoder = new CANcoder(m_motor.getDeviceID(), "Canivore");

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
		config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		config.Feedback.SensorToMechanismRatio = 1;
		config.Feedback.RotorToSensorRatio = 100;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.920654;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.3 ;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.CurrentLimits.SupplyCurrentLimit = 80;
		config.CurrentLimits.SupplyCurrentLimitEnable = false;
		config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
		config.CurrentLimits.StatorCurrentLimitEnable = false;

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		encoderConfig.MagnetSensor.MagnetOffset = Units.degreesToRotations(-0.12866);

		m_motor.getConfigurator().apply(config);
		m_encoder.getConfigurator().apply(encoderConfig);
		// m_motor.setInverted(isLeft);

		m_motor.getPosition().setUpdateFrequency(50);
		m_motor.optimizeBusUtilization();

		m_Servo = new Servo(servoChannel);

	}

	public double getMotorRotations() {
		return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putNumber("Climber Encoder Position", m_encoder.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Servo Position", m_Servo.getPosition());
		SmartDashboard.putNumber("Climber Position", m_motor.getPosition().getValueAsDouble());

	}

	public void engageRatchet() {

		m_Servo.setPosition(0.5);

	}

	public void disengageRatchet() {

		m_Servo.setPosition(0.7);
	}

	public void setVoltage(double voltage) {

		m_motorRequest.withOutput(voltage);
		m_motor.setControl(m_motorRequest);
	}

	public void stop() {
		setVoltage(0);
		
	}

	public double getPosition(){
		return m_encoder.getPosition().getValueAsDouble();
	}


	public Command retractCommand() {
		return new ParallelCommandGroup(
				// Commands.run(() -> engageRatchet(),));
				new ParallelCommandGroup(
				// Commands.runEnd(() -> disengageRatchet(), this::engageRatchet)),
				Commands.runEnd(() -> this.setVoltage(8), this::stop, this))

		);
	}

	public Command extendCommand(){
		return new ParallelCommandGroup(
			Commands.runEnd(() -> disengageRatchet(), this::engageRatchet),
			Commands.runEnd(() -> this.setVoltage(-4), this::stop, this)

	// new SequentialCommandGroup(
	// 	new WaitCommand(0.2)
		// Commands.runEnd(() -> this.setVoltage(4), this::stop, this)
	);
}

}
