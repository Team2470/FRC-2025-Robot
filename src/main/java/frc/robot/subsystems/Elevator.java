// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private enum ControlMode {
    kOpenLoop, kPID, kStop, kHoming, kCoast
  }

  //
  // Hardware
  //
  private final TalonFX m_motor;
  private final TalonFX m_motorFollower;
  private final DigitalInput m_retractLimit;
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final CoastOut m_CoastOut = new CoastOut();

  //
  // State
  //
  private ControlMode m_controlMode = ControlMode.kStop;
  private final ProfiledPIDController m_pidController = new ProfiledPIDController(ElevatorConstants.kP,
      ElevatorConstants.kI, ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(50, 150));

  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG,
      ElevatorConstants.kV,
      ElevatorConstants.kA);

  private double m_pidLastVelocitySetpoint = 0;
  private double m_pidLastTime;

  private double m_demand;
  private boolean m_isHomed;

  public Elevator() {

    m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;

    //
    // Setup Follower motor
    //
    m_motorFollower = new TalonFX(ElevatorConstants.kMotorFollowerID, "Canivore");
    m_motorFollower.getConfigurator().apply(m_motorConfig);
    m_motorFollower.optimizeBusUtilization();
    m_motorFollower.setControl(new Follower(ElevatorConstants.kMotorID, true));

    //
    // Apply add extra leader configuration on top of the base config
    // - Any control related settings for PID, Motion Magic, Remote Sensors, etc..
    // - Soft limits should only be applied to the leader motor, if they are applied
    // to the follower
    // it may stop moving if their built in encoders are not in sync.
    //
    m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 111;
    m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1;

    m_motor = new TalonFX(ElevatorConstants.kMotorID, "Canivore");
    m_motor.getConfigurator().apply(m_motorConfig);

    // We want to read position data from the leader motor
    m_motor.getPosition().setUpdateFrequency(50);

    // These 3 are needed for the follower motor to work
    m_motor.getDutyCycle().setUpdateFrequency(50);
    m_motor.getMotorVoltage().setUpdateFrequency(50);
    m_motor.getTorqueCurrent().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();

    // Limit switch setup
    m_retractLimit = new DigitalInput(ElevatorConstants.kRetractLimitChannel);

    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
    SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
    SmartDashboard.putNumber("Elevator kS", ElevatorConstants.kS);
    SmartDashboard.putNumber("Elevator kG", ElevatorConstants.kG);
    SmartDashboard.putNumber("Elevator kV", ElevatorConstants.kV);
    SmartDashboard.putNumber("Elevator kA", ElevatorConstants.kA);

  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble() * ElevatorConstants.kRotationToInches;
  }

  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble() * ElevatorConstants.kRotationToInches;
  }

  public boolean isRetracted() {
    return !m_retractLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double outputVoltage = 0;

    if (!m_isHomed && isRetracted()) {
      m_motor.setPosition(0);
      m_isHomed = true;
    }
    SmartDashboard.putNumber("Elevator Wanted Demand", m_demand);
    SmartDashboard.putString("Elevator Wanted Control Mode", m_controlMode.toString());
    if (!m_isHomed && m_controlMode != ControlMode.kHoming) {
      stop();
    }

    switch (m_controlMode) {

      case kHoming:

        // Do homing stuff here
        outputVoltage = -2;

        break;
      case kStop:

        outputVoltage = 0;
        break;

      case kOpenLoop:
        // Do openloop stuff here

        outputVoltage = m_demand;

        break;

      case kPID:

        m_feedforward = new ElevatorFeedforward(
            SmartDashboard.getNumber("Elevator kS", ElevatorConstants.kS),
            SmartDashboard.getNumber("Elevator kG", ElevatorConstants.kG),
            SmartDashboard.getNumber("Elevator kV", ElevatorConstants.kV),
            SmartDashboard.getNumber("Elevator kA", ElevatorConstants.kA));

        m_pidController.setP(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kP));
        m_pidController.setI(SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kI));
        m_pidController.setD(SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kD));
        
        double PIDoutPutVoltage = m_pidController.calculate(getPosition(), m_demand);

        double dt = Timer.getFPGATimestamp() - m_pidLastTime;
        double accelerationSetpoint = (m_pidController.getSetpoint().velocity - m_pidLastVelocitySetpoint) / dt;
        double feedforwardVoltage = m_feedforward.calculate(m_pidController.getSetpoint().velocity,
            accelerationSetpoint);

        outputVoltage = PIDoutPutVoltage + feedforwardVoltage;
        SmartDashboard.putNumber("Elevator Pid output voltage", PIDoutPutVoltage);
        SmartDashboard.putNumber("Elevator Feed Fowrad output voltage", feedforwardVoltage);
        SmartDashboard.putNumber("Elevator PID Profile Position",m_pidController.getSetpoint().position);
        SmartDashboard.putNumber("Elevator PID Profile Velocity",m_pidController.getSetpoint().velocity);

        m_pidLastVelocitySetpoint = m_pidController.getSetpoint().velocity;
        m_pidLastTime = Timer.getFPGATimestamp();

        break;
      default:
        // What happened!?
        break;
    }

    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    SmartDashboard.putNumber("Elevator Demand", m_demand);
    SmartDashboard.putNumber("Elevator PositionInRotations", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator is Retracted", isRetracted());
    SmartDashboard.putBoolean("Elevator is Homed", m_isHomed);
    SmartDashboard.putString("Elevator Controlmode", m_controlMode.toString());
    SmartDashboard.putNumber("Elevator Demand", m_demand);

    m_motor.setVoltage(outputVoltage);

    if (m_controlMode != ControlMode.kCoast) {
      m_motor.setVoltage(outputVoltage);

  } else {
      m_motor.setControl(m_CoastOut);
  }

  }
  

  public Command coastCommand() {
    return Commands.runEnd(
        () ->{
            m_controlMode = ControlMode.kCoast;
            m_demand = 0;
        }, this::stop, this);

}

  public void setOutputVoltage(double OutputVoltage) {
    m_controlMode = ControlMode.kOpenLoop;
    m_demand = OutputVoltage;
  }

  public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {
    return Commands.runEnd(
        () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

  }

  public Command openLoopCommand(double OutputVoltage) {
    return openLoopCommand(() -> OutputVoltage);
  }

  public Command elevatorHomeCommand() {
    return new FunctionalCommand(
        () -> {
          m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
          m_motor.getConfigurator().apply(m_motorConfig);
        },
        () -> {
          m_controlMode = ControlMode.kHoming;
        },
        (Boolean) -> {
          m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          m_motor.getConfigurator().apply(m_motorConfig);
          stop();
        }, this::isRetracted,
        this);
  }

  public void stop() {
    m_controlMode = ControlMode.kStop;
    m_demand = 0;
  }

  // pid

  public boolean isErrorInRange() {
    // Todo: Find the correct value
    return (-4 < this.getErrorPercent() && this.getErrorPercent() < 4);
  }

  public double getErrorPercent() {
    if (m_controlMode == ControlMode.kPID) {
      return (m_demand - getPosition()) / m_demand * 10;
    }
    return 0;
  }

  public Command waitUntilErrorInrange() {
    return Commands.waitUntil(() -> this.isErrorInRange());
  }

  public boolean isErrorOutOfRange() {
    return (this.getErrorPercent() > 15);
  }

  public Command waitUntilErrorOutOfRange() {
    return Commands.waitUntil(() -> this.isErrorOutOfRange());

  }

  public void setPIDSetpoint(double inches) {
    if (m_controlMode != ControlMode.kPID) {
      m_pidController.reset(getPosition());
      m_pidLastVelocitySetpoint = 0;
      m_pidLastTime = Timer.getFPGATimestamp();
    }
    m_controlMode = ControlMode.kPID;
    m_demand = inches;
  }

  public Command pidCommand(DoubleSupplier inches) {
    return Commands.runEnd(
        () -> this.setPIDSetpoint(inches.getAsDouble()), this::stop, this);
  }

  public Command pidCommand(double inches) {
    return pidCommand(() -> inches);
  }

}
