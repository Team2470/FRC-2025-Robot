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
  PoseEstimate lastEstimateMiddle = new PoseEstimate();


  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newRightEstimate = false;
  @NotLogged
  boolean newLeftEstimate = false;
  @NotLogged
  boolean newMiddleEstimate = false;

  Pose2d rightPose = new Pose2d();
  Pose2d leftPose = new Pose2d();
  Pose2d middlePose = new Pose2d();


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
    PoseEstimate currentEstimateMiddle = new PoseEstimate();


    if (useMegaTag2) {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      currentEstimateMiddle = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-middle");
    } else {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
      currentEstimateMiddle = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-middle");
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
    if (currentEstimateMiddle != null && !rejectUpdate(currentEstimateMiddle, gyroRate)) {
      lastEstimateMiddle = currentEstimateMiddle;
      middlePose = currentEstimateMiddle.pose;
      newMiddleEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
      if (!newRightEstimate && !newLeftEstimate && !newMiddleEstimate) {
        return Optional.empty();
    
    } else if (!newRightEstimate && !newLeftEstimate && newMiddleEstimate) {
        // One valid pose estimate (middle)
        newMiddleEstimate = false;
        return Optional.of(lastEstimateMiddle);
    
    } else if (!newRightEstimate && newLeftEstimate && !newMiddleEstimate) {
        // One valid pose estimate (left)
        newLeftEstimate = false;
        // return Optional.of(lastEstimateLeft);
        return Optional.empty();
    
    } else if (newRightEstimate && !newLeftEstimate && !newMiddleEstimate) {
        // One valid pose estimate (right)
        newRightEstimate = false;
        // return Optional.of(lastEstimateRight);
        return Optional.empty();
    
    } else {
        // More than one valid pose estimate, discard the one that's furthest
        newRightEstimate = false;
        newLeftEstimate = false;
        newMiddleEstimate = false;
    
        if (lastEstimateLeft.avgTagDist < lastEstimateRight.avgTagDist &&
            lastEstimateLeft.avgTagDist < lastEstimateMiddle.avgTagDist) {
            return Optional.of(lastEstimateLeft);
        } else if (lastEstimateRight.avgTagDist < lastEstimateMiddle.avgTagDist) {
            return Optional.of(lastEstimateRight);
        } else {
            return Optional.of(lastEstimateMiddle);
        }
    }
    
  }

  @Override
  public void periodic() {
  }



  
}
