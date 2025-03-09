// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeServo extends SubsystemBase {
  /** Creates a new IntakeServo. */
  private final Servo m_servo;
  private final boolean m_isLeft;

  public IntakeServo(int servoChannel, boolean isLeft) {
    m_servo = new Servo(servoChannel);
    m_isLeft = isLeft;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("servo pos" + m_isLeft , m_servo.getPosition());
  }

  private void engage() {
    if (m_isLeft) {
      m_servo.setPosition(0.4);
    } else {
    m_servo.setPosition(0.6);
  }
}

  private void disengage() {
    if (m_isLeft) {
      m_servo.setPosition(0.6);
    } else {
    m_servo.setPosition(0.4);
  }
  }

  public Command engageServo(){
    return Commands.run(() -> engage());
  }

  public Command disengageServo(){
    return Commands.run(() -> disengage());
  }
}

