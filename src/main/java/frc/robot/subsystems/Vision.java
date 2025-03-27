// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateRight = new PoseEstimate();
  PoseEstimate lastEstimateLeft = new PoseEstimate();

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newRightEstimate = false;
  @NotLogged
  boolean newLeftEstimate = false;

  Pose2d rightPose = new Pose2d();
  Pose2d leftPose = new Pose2d();

  private boolean useMegaTag2 = false;
  public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

  public Vision() {
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] { lastEstimateRight, lastEstimateLeft };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * 
   * @return True if the estimate should be rejected
   */

  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > 0.1) {//their value
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimateRight = new PoseEstimate();
    PoseEstimate currentEstimateLeft = new PoseEstimate();

    if (useMegaTag2) {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    } else {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    }

    if (currentEstimateRight != null && !rejectUpdate(currentEstimateRight, gyroRate)) {
      lastEstimateRight = currentEstimateRight;
      rightPose = currentEstimateRight.pose;
      newRightEstimate = true;
    }
    if (currentEstimateLeft != null && !rejectUpdate(currentEstimateLeft, gyroRate)) {
      lastEstimateLeft = currentEstimateLeft;
      leftPose = currentEstimateLeft.pose;
      newLeftEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newRightEstimate && !newLeftEstimate) {
      return Optional.empty();

    } else if (newRightEstimate && !newLeftEstimate) {
      // One valid pose estimate (right)
      newRightEstimate = false;
      return Optional.of(lastEstimateRight);

    } else if (!newRightEstimate && newLeftEstimate) {
      // One valid pose estimate (left)
      newLeftEstimate = false;
      return Optional.of(lastEstimateLeft);

    } else {
      // Two valid pose estimates, disgard the one that's further
      newRightEstimate = false;
      newLeftEstimate = false;
      if (lastEstimateLeft.avgTagDist < lastEstimateRight.avgTagDist) {
        return Optional.of(lastEstimateRight);
      } else {
        return Optional.of(lastEstimateLeft);
      }
    }
  }

  @Override
  public void periodic() {
  }
}
