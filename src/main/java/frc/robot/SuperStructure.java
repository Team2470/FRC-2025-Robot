package frc.robot;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SuperStructure {
    
    public enum State {
        kDrivePosition,
        k
    }

    public class EdgeCommand extends DefaultEdge {
        
    }

    private final Graph<State, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;

    public SuperStructure(Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
    }

}
