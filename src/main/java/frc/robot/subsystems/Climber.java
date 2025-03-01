// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

public class Climber extends SubsystemBase {

private final TalonFX m_motor ;
private final VoltageOut m_motorRequest = new VoltageOut(0);

private final Servo m_Servo;



/** Creates a new ClimberPivot. */
public Climber(int motorID, int servoChannel) {

	TalonFXConfiguration config = new TalonFXConfiguration();

	config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

	config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	config.CurrentLimits.SupplyCurrentLimit = 20;
	config.CurrentLimits.SupplyCurrentLimitEnable = true;
	config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
	config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 34;
	config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
	config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
	config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;


	m_motor = new TalonFX(motorID, "rio");
	m_motor.getConfigurator().apply(config);
	//m_motor.setInverted(isLeft);

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
	
	



}

public void engageRatchet(){

	m_Servo.setPosition(0.475);


}
public void disengageRatchet(){

	m_Servo.setPosition(0.35);
}

public void setVoltage(double voltage){
	

	m_motorRequest.withOutput(voltage);
	m_motor.setControl(m_motorRequest);
}

public void stop(){
	setVoltage(0);
}

public Command retractCommand(){
	return new ParallelCommandGroup(
	Commands.run(() -> engageRatchet()),
	Commands.runEnd(() -> this.setVoltage(-3), this::stop, this));
}

	public Command extendCommand(){
	return new ParallelCommandGroup(
	Commands.run(() -> disengageRatchet()),
	new SequentialCommandGroup(
		new WaitCommand(0.2),
		Commands.runEnd(() -> this.setVoltage(4), this::stop, this)
	)
	);
}

}
