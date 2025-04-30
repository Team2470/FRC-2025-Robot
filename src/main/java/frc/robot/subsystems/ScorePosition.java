// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScorePosition extends SubsystemBase {
  /** Creates a new ScorePosition. */


  private double Position = 4;

  public ScorePosition() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private void AddPosition(){
    Position = Position++;
  }

  public Command AddPosistion(){
    return Commands.runOnce(()->AddPosition() , this);
  }

  private void MinusPosition(){
    Position = Position--; 
  }
  public Command MinusPosistion(){
    return Commands.runOnce(()->MinusPosition() , this);
  }

  public double getScorePosition(){
    return Position;
  }
}
