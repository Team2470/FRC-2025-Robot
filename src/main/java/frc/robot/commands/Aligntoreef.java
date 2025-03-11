// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.pheonix6.swerve.ModifiedRobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.Aligntoreef.Side;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aligntoreef extends SequentialCommandGroup {

  public enum Side {
    Left("limelight-left"),
    Right("limelight-right");

    public final String name;
    private Side(String name) {
      this.name = name;
    }
  }
  public enum Score {
    Coral(0),
    Algae(1);

    public final int pipeline;
    private Score(int pipeline) {
      this.pipeline = pipeline;
    }
  }
  private final PIDController m_txPID = new PIDController(0.2, 0, 0.002);
  private final PIDController m_tyPID = new PIDController(0.2, 0, 0);
  private double xFeedForward;
  private double yFeedForward;
  private double heading;

  final ModifiedRobotCentricFacingAngle swerveAlign = new ModifiedRobotCentricFacingAngle()
    .withHeadingPID(10, 0, 0)
    .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Creates a new Aligntoreef. */
  public Aligntoreef(CommandSwerveDrivetrain drive, Side side, Score score) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    setName("Aligntoreef");
    addCommands(
      Commands.runOnce(() -> {
        LimelightHelpers.setPipelineIndex(side.name, score.pipeline);
      }),
      new WaitUntilCommand(()-> LimelightHelpers.getTV(side.name)),
      Commands.runOnce(() -> {
        m_txPID.reset();
        m_txPID.setP(0.2);
        m_txPID.setI(0);
        m_txPID.setD(0.002);
        m_txPID.setTolerance(0.6);

        m_tyPID.reset();
        m_tyPID.setP(0.2);
        m_tyPID.setI(0);
        m_tyPID.setD(0);
        m_tyPID.setTolerance(0.6);

        if (LimelightHelpers.getTX(side.name) <= 0) {
          yFeedForward = 0.11;
        } else  {
          yFeedForward = -0.11;
        }

        if (LimelightHelpers.getTY(side.name) <= 0) {
          xFeedForward = 0.11;
        } else {
          xFeedForward = -0.11;
        } 

        int tagID = (int) LimelightHelpers.getFiducialID(side.name);


        switch (tagID) {
          case 10: heading = 0; break;
          case 18: heading = 0; break;
          case 11: heading = 60; break;
          case 17: heading = 60; break;
          case 7: heading = 180; break;
          case 21: heading = 180; break;
          case 8: heading = 60; break;
          case 20: heading = 240; break;
          case 9: heading = 300; break;
          case 19: heading = 300; break;

          default:
            // This is a tag that we can't handle! OH NO
        }

      }),
      drive.applyRequest(() -> {
        double xMove = MathUtil.clamp(
          m_tyPID.calculate(LimelightHelpers.getTY(side.name), 0), -0.2,0.8
        );
        double yMove = MathUtil.clamp(
          m_txPID.calculate(LimelightHelpers.getTX(side.name), 0), -0.2,0.8
        );

        //yMove = 0.11;
        xMove += xFeedForward;
        yMove += yFeedForward;
        // xMove = MathUtil.applyDeadband(xMove, 0.01);
        // yMove = MathUtil.applyDeadband(yMove, 0.01);
        if (m_tyPID.atSetpoint()) {
          xMove=0;
        }
        SmartDashboard.putNumber("AlignToReef Heading Error", swerveAlign.HeadingController.getPositionError());
        SmartDashboard.putNumber("AlignToReef Heading Setpoint", swerveAlign.HeadingController.getSetpoint());
        SmartDashboard.putNumber("AlignToReef tx error", m_txPID.getError());
        SmartDashboard.putNumber("AlignToReef ty error", m_tyPID.getError());
        SmartDashboard.putNumber("AlignToReef xFeedForward", xFeedForward);
        SmartDashboard.putNumber("AlignToReef yFeedForward", yFeedForward);
        SmartDashboard.putNumber("AlignToReef xMove", xMove);
        SmartDashboard.putNumber("AlignToReef yMove", yMove);
        SmartDashboard.putBoolean("AlignToReef isFarAway", true);
        return swerveAlign
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withTargetDirection(Rotation2d.fromDegrees(heading));
      }).until(m_tyPID::atSetpoint),
      Commands.runOnce(() -> {
        m_txPID.setP(0.1);
        m_txPID.setI(0);
        m_txPID.setD(0.002);
        m_txPID.setTolerance(0.6);

        m_tyPID.setP(0.2);
        m_tyPID.setI(0);
        m_tyPID.setD(0);
        m_tyPID.setTolerance(0.6);


      }),
      drive.applyRequest(() -> {
        // double xMove = MathUtil.clamp(
        //   m_tyPID.calculate(LimelightHelpers.getTY(side.name), 0), -0.2,0.5
        // );
        double yMove = MathUtil.clamp(
          m_txPID.calculate(LimelightHelpers.getTX(side.name), 0), -0.2,0.5
        );
        double xMove = 0;
        //yMove = 0.11;
        // xMove += xFeedForward;
        // yMove += yFeedForward;
        // xMove = MathUtil.applyDeadband(xMove, 0.01);
        // yMove = MathUtil.applyDeadband(yMove, 0.01);
        SmartDashboard.putNumber("AlignToReef Heading Error", swerveAlign.HeadingController.getPositionError());
        SmartDashboard.putNumber("AlignToReef Heading Setpoint", swerveAlign.HeadingController.getSetpoint());
        SmartDashboard.putNumber("AlignToReef tx error", m_txPID.getError());
        SmartDashboard.putNumber("AlignToReef ty error", m_tyPID.getError());
        SmartDashboard.putNumber("AlignToReef xFeedForward", xFeedForward);
        SmartDashboard.putNumber("AlignToReef yFeedForward", yFeedForward);
        SmartDashboard.putNumber("AlignToReef xMove", xMove);
        SmartDashboard.putNumber("AlignToReef yMove", yMove);
         SmartDashboard.putBoolean("AlignToReef isFarAway", false);
        
        return swerveAlign
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withTargetDirection(Rotation2d.fromDegrees(heading));

      })

    );
  }
  public static Command autoCoralSide(CommandSwerveDrivetrain drive, Side userSide) {
    return Commands.sequence(
      new WaitUntilCommand(() -> LimelightHelpers.getTV(Side.Left.name) || LimelightHelpers.getTV(Side.Right.name)),
      new ConditionalCommand(
        new Aligntoreef(drive, Side.Right,Score.Coral),
        new Aligntoreef(drive, Side.Left, Score.Coral),
        () -> {
          boolean alliance;
          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            alliance = true;
          } else {
            alliance = false;
          }
          
          int tagid;
          if (LimelightHelpers.getTA(Side.Right.name) > LimelightHelpers.getTA(Side.Left.name)) {       
            // tagSide = Side.Right;
            tagid = (int) LimelightHelpers.getFiducialID(Side.Right.name);
          } else {
            // tagSide = Side.Left;
            tagid = (int) LimelightHelpers.getFiducialID(Side.Left.name);
          }
          if (alliance) {
            // red alliance
            switch (tagid) {
              // case 10: inverted = true;
              // case 18: inverted = true; break;
              // case 11: inverted = true; break;
              // case 17: inverted = true; break;
              // case 7: inverted = false; break;
              // case 21: inverted = false; break;
              // case 8:  inverted = false; break;
              // case 20: inverted = false; break;
              // case 9: inverted = true; break;
              // case 19: inverted = true; break;


              case 10: return true;
              case 18: return true;
              case 11: return true; 
              case 17: return true; 
              case 7: return false; 
              case 21: return false; 
              case 8:  return false; 
              case 20: return false; 
              case 9: return true; 
              case 19: return true; 
    

              default:
                // This is a tag that we can't handle! OH NO
                return false;
            }
          } else {
            switch (tagid) {
              case 10: return false;
              case 18: return false; 
              case 11: return false; 
              case 17: return false; 
              case 7: return true; 
              case 21: return true; 
              case 8: return true; 
              case 20: return true; 
              case 9: return false; 
              case 19: return false; 
    
              default:
                return false;
              // This is a tag that we can't handle! OH NO
            }
          }
        }
        // TRUE
        // if ((userSide == tagSide) && !inverted) {
        //   new Aligntoreef(drive, tagSide, Score.Coral);
        // } else if ((userSide != tagSide)) {

        // }
      )
    );
  }
}