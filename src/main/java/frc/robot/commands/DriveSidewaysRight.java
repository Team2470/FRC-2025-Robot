package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveSidewaysRight extends SequentialCommandGroup {
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(-0.45)
            .withRotationalRate(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private Pose2d startPose;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public DriveSidewaysRight(CommandSwerveDrivetrain drivetrain, double distanceMeters) {
        addCommands(
            new InstantCommand(() -> startPose = drivetrain.getState().Pose),
            drivetrain.applyRequest(()-> {
                SmartDashboard.putNumber("DriveStraight", (drivetrain.getState().Pose.minus(startPose).getTranslation()).getNorm());
                return request;
            }).finallyDo(()-> drivetrain.setControl(idleRequest)).until(()-> (drivetrain.getState().Pose.minus(startPose).getTranslation()).getNorm() > distanceMeters)
        );
    }

    

}
