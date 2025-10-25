// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AutoCommon;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HumanIntake;
import frc.robot.subsystems.InnerIntake;
import frc.robot.subsystems.OuterIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Superstructure.*;
import frc.robot.subsystems.Superstructure.Superstructure.m_State;
import frc.robot.subsystems.Superstructure.SuperstructurePosition.TargetAction;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public abstract class AutoBase extends SequentialCommandGroup {
    private final OuterIntake algea = new OuterIntake(42, 44, false);
    private final InnerIntake coral = new InnerIntake(41, 43, false);
    private final HumanIntake intake = new HumanIntake(45, 46, true);

    protected final Wrist wrist = new Wrist();
    protected final Elevator elevator1 = new Elevator(intake.getCANDI());
    protected final Arm arm = new Arm(algea.getCANDI());
    
    protected final AutoFactory autoFactory = AutoFactory.getInstance();
    protected final Superstructure superstructure = new Superstructure(arm, elevator1, wrist);
    
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private Pose2d startPose;

    protected AutoBase(Optional<Pose2d> pathStartPose) {
        if (pathStartPose.isEmpty()) {
            startPose = new Pose2d();
        } else {
            startPose = pathStartPose.get();
        }

        if (RobotState.getInstance().isRedAlliance()) {
            startPose = FlippingUtil.flipFieldPose(startPose);
        }

        setStartPose(startPose);
        RobotState.getInstance().setAutoStartPose(startPose);
    }

    public abstract void init(); // defined in each Auto class

    private void setStartPose(Pose2d pathStartPose) {
        addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
    }

    protected Command manualZero() {
    final var fieldCentric = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    final var fieldCentricIdle = new SwerveRequest.Idle();
    }

    protected Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    protected Command startHP() {
    }

    protected static PathPlannerPath getPathFromFile(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET PATH FROM PATHFILE" + pathName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static Optional<Pose2d> getStartPoseFromAutoFile(String autoName) {
    }

    protected Command getBumpCommand() {
    }

    protected Command delaySelectedTime() {
    }

    protected Command elevatorToPos(TargetAction position) {
        return new InstantCommand( () -> superstructure.setCurrentAction(position));
    }

    protected Command score(TargetAction position) {
    }

    protected BooleanSupplier haveCoral() {
    }

    protected Command toPosAndScore(TargetAction position) {
    }

    protected Command pickup(Path path) {
    }

    protected Command toPosition(m_State pos) {
        superstructure.setRobotState(pos);

    }

    protected Command autoDrivePositiCommand() {
        superstructure.setRobotState(m_State.Drive);
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    arm.pidCommand(82),
                    wrist.pidCommand(85)).until(() -> Math.abs(arm.getPosition() - 77) < 3),
                
                wrist.pidCommand(85).until(() -> Math.abs(wrist.getPosition() - 85) < 10),
                () -> elevator1.getPosition() > 30),
            
            arm.pidCommand(77).until(() -> Math.abs(arm.getPosition() - 77) < 3),
            
            new ParallelCommandGroup(
                arm.pidCommand(77),
                wrist.pidCommand(85),
                elevator1.pidCommand(1).until(() -> Math.abs(elevator1.getErrorPercent()) < 2)
    
            )).withName("Auto Drive Position");
      }

    public static class Path { // combines access to pathplanner and choreo
        private String pathPlannerPathName;

        public Path(String PPName) {
            pathPlannerPathName = PPName;
        }

        public PathPlannerPath getPathPlannerPath() {
            try {
                return getPathFromFile(pathPlannerPathName);

            } catch (Exception e) {
                DriverStation.reportError(
                        "FAILED TO GET PATH FROM PATHFILE " + pathPlannerPathName + e.getMessage(), e.getStackTrace());
                return null;
            }
        }
    }

    public static final class PathsBase {
        // Paths for starting Autos
        public static final Path leftAutoStart      = new Path("leftAutoStart");
        public static final Path middleAutoStart    = new Path("middleAutoStart");
        public static final Path rightAutoStart     = new Path("rightAutoStart");

        // Paths that go from backed up position to scoring at post posistion
        public static final Path abToA              = new Path("abToA");
        public static final Path abToB              = new Path("abToB");
        public static final Path cdToC              = new Path("cdToC");
        public static final Path cdToD              = new Path("cdToD");
        public static final Path efToE              = new Path("efToE");
        public static final Path efToF              = new Path("efToF");
        public static final Path ghToG              = new Path("ghToG");
        public static final Path ghToH              = new Path("ghToH");
        public static final Path ijToI              = new Path("ijToI");
        public static final Path ijToJ              = new Path("ijToJ");
        public static final Path klToK              = new Path("klToK");
        public static final Path klToL              = new Path("klToL");

        // Paths that go from post scoring position to backed up position
        public static final Path aToAB              = new Path("aToAB");
        public static final Path bToAB              = new Path("bToAB");
        public static final Path cToCD              = new Path("cToCD");
        public static final Path dToCD              = new Path("dToCD");
        public static final Path eToEF              = new Path("eToEF");
        public static final Path fToEF              = new Path("fToEF");
        public static final Path gToGH              = new Path("gToGH");
        public static final Path hToGH              = new Path("hToGH");
        public static final Path iToIJ              = new Path("iToIJ");
        public static final Path jToIJ              = new Path("jToIJ");
        public static final Path kToKL              = new Path("kToKL");
        public static final Path lToKL              = new Path("lToKL");

        // Paths that go from backed up position to Left HP Station
        public static final Path abToLeftHP         = new Path("abToLeftHP");
        public static final Path cdToLeftHP         = new Path("cdToLeftHP");
        public static final Path efToLeftHP         = new Path("efToLeftHP");
        public static final Path ghToLeftHP         = new Path("ghToLeftHP");
        public static final Path ijToLeftHP         = new Path("ijToLeftHP");
        public static final Path klToLeftHP         = new Path("klToLeftHP");

        // Paths that go from backed up position to Right HP Station
        public static final Path abToRightHP        = new Path("abToRightHP");
        public static final Path cdToRightHP        = new Path("cdToRightHP");
        public static final Path efToRightHP        = new Path("efToRightHP");
        public static final Path ghToRightHP        = new Path("ghToRightHP");
        public static final Path ijToRightHP        = new Path("ijToRightHP");
        public static final Path klToRightHP        = new Path("klToRightHP");

        // Paths from Left HP Station to backed up position
        public static final Path leftHPToAB         = new Path("leftHPToAB");
        public static final Path leftHPToCD         = new Path("leftHPToCD");
        public static final Path leftHPToEF         = new Path("leftHPToEF");
        public static final Path leftHPToGH         = new Path("leftHPToGH");
        public static final Path leftHPToIJ         = new Path("leftHPToIJ");
        public static final Path leftHPToKL         = new Path("leftHPToKL");

        // Paths from Right HP Station to backed up position
        public static final Path rightHPToAB        = new Path("rightHPToAB");
        public static final Path rightHPToCD        = new Path("rightHPToCD");
        public static final Path rightHPToEF        = new Path("rightHPToEF");
        public static final Path rightHPToGH        = new Path("rightHPToGH");
        public static final Path rightHPToIJ        = new Path("rightHPToIJ");
        public static final Path rightHPToKL        = new Path("rightHPToKL");
    }
}