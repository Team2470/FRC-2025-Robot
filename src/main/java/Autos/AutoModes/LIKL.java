package Autos.AutoModes;
import java.nio.file.Paths;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;

import Autos.AutoCommon.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HumanIntake;
import frc.robot.subsystems.InnerIntake;
import frc.robot.subsystems.OuterIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Superstructure.*;
import frc.robot.subsystems.Superstructure.SuperstructurePosition.TargetAction;
import frc.robot.*;

public class LIKL extends AutoBase {
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
    private static final Path startPath = PathsBase.leftAutoStart;

    public LIKL() {
        super(startPath.getPathPlannerPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {

        // Get from Left Auto Start position to I/J Reef face
        addCommands(
                followPathCommand(startPath.getPathPlannerPath())
                .deadlineFor(autoDrivePositiCommand()));
        
        // Align to reef and get superstructure to L4 position
        addCommands(
                followPathCommand(PathsBase.ijToI.getPathPlannerPath())
                .deadlineFor(reefL4Command()));

        // Score at L4
        addCommands(toPosAndScore(TargetAction.L4));    // Holds elevator, arm and wrist at l4 position while scoring
        
        // Back up to reef face, droping to drive position
        addCommands(
                followPathCommand(PathsBase.iToIJ.getPathPlannerPath())
                .andThen(autoDrivePositiCommand()));

        // Get from I/J Reef face to left HP station
        addCommands(
                followPathCommand(PathsBase.ijToLeftHP.getPathPlannerPath())
                .deadlineFor(HumanPlayerIntakeCommand()));

        // Get from left HP station to K/L Reef face
        addCommands(
                followPathCommand(PathsBase.leftHPToKL.getPathPlannerPath())
                .deadlineFor(autoDrivePositiCommand()));
        
        // Get from K/L Reef face to K Reef face
        addCommands(
                followPathCommand(PathsBase.klToK.getPathPlannerPath())
                .deadlineFor(reefL4Command()));
        
        // Score at L4
        addCommands(toPosAndScore(TargetAction.L4)); 

        // Back up to reef face, droping to drive position
        addCommands(
                followPathCommand(PathsBase.kToKL.getPathPlannerPath())
                .andThen(autoDrivePositiCommand()));
    
        // Get from K/L Reef face to left HP station
        addCommands(
                followPathCommand(PathsBase.klToLeftHP.getPathPlannerPath())
                .deadlineFor(HumanPlayerIntakeCommand()));

        // Get from left HP station to K/L Reef face
        addCommands(
                followPathCommand(PathsBase.leftHPToKL.getPathPlannerPath())
                .deadlineFor(autoDrivePositiCommand()));

        // Get from K/L Reef face to K Reef face
        addCommands(
                followPathCommand(PathsBase.klToL.getPathPlannerPath())
                .deadlineFor(reefL4Command()));
        
        // Score at L4
        addCommands(toPosAndScore(TargetAction.L4)); 

        // Back up to reef face, droping to drive position
        addCommands(
                followPathCommand(PathsBase.lToKL.getPathPlannerPath())
                .andThen(autoDrivePositiCommand()));

//  public PathfindThenFollowPath(
//      PathPlannerPath                             goalPath,
//      PathConstraints                             pathfindingConstraints,
//      Supplier<Pose2d>                            poseSupplier,
//      Supplier<ChassisSpeeds>                     currentRobotRelativeSpeeds,
//      BiConsumer<ChassisSpeeds,DriveFeedforwards> output,
//      PathFollowingController                     controller,
//      RobotConfig                                 robotConfig,
//      BooleanSupplier                             shouldFlipPath,
//      Subsystem...                                requirements
// )
    }
}
