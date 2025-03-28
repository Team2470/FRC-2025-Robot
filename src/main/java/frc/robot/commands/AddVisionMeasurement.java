// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {
  CommandSwerveDrivetrain subDrivetrain;
  Vision subVision;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(CommandSwerveDrivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;

    addRequirements(subVision);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    var driveState = subDrivetrain.getState();
    // double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    // Tells the limelight where we are on the field
    LimelightHelpers.SetRobotOrientation("limelight-right",
        driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left",
        driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = DegreesPerSecond.of(omegaRps);

    Optional<PoseEstimate> estimatedPose = subVision.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent()) {
      subDrivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds, VecBuilder.fill(.5,.5,9999999));
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
