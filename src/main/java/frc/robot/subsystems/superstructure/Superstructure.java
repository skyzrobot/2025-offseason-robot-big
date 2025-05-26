package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {


    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    @Getter private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter private SuperstructureState goal = SuperstructureState.START;
    private EdgeCommand edgeCommand;
    private final IntakeSubsystem intake;
    private final EndEffectorArmSubsystem endEffectorArm;
    private final ElevatorSubsystem elevator;
    private final SuperstructureVisualizer currentPoseVisualizer;
    private final SuperstructureVisualizer setpointPoseVisualizer;
    private final SuperstructureVisualizer goalPoseVisualizer;

    /**
     * Constructor for the Superstructure subsystem.
     * 
     * Important Implementation Requirements:
     * 1. All state transitions (edges) must be declared in the constructor using addEdge()
     * 2. All edge commands must be implemented in getEdgeCommand()
     * 3. All commands must properly exit and finish - they should not run indefinitely
     *    This is crucial for the state machine to progress to the next state
     * 
     * @see #addEdge(SuperstructureState, SuperstructureState, boolean, boolean)
     * @see #getEdgeCommand(SuperstructureState, SuperstructureState)
     * @see EdgeCommand
     */
    public Superstructure(IntakeSubsystem intake, EndEffectorArmSubsystem endEffectorArm, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.endEffectorArm = endEffectorArm;
        this.elevator = elevator;
        this.currentPoseVisualizer = new SuperstructureVisualizer("Current");
        this.setpointPoseVisualizer = new SuperstructureVisualizer("Setpoint");
        this.goalPoseVisualizer = new SuperstructureVisualizer("Goal");

        // Add states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }

        // Declear all edges here
        addEdge(SuperstructureState.START, SuperstructureState.IDLE, false, false);
        // Add edges between shoot and preshoot states
        final Set<Pair<SuperstructureState, SuperstructureState>> shootStates =
        Set.of(
            Pair.of(SuperstructureState.L1_INTAKE_SIDE, SuperstructureState.L1_INTAKE_SIDE_EJECT),
            Pair.of(SuperstructureState.L1_SHOOT_SIDE, SuperstructureState.L1_SHOOT_SIDE_EJECT),
            Pair.of(SuperstructureState.L2, SuperstructureState.L2_EJECT),
            Pair.of(SuperstructureState.L3, SuperstructureState.L3_EJECT),
            Pair.of(SuperstructureState.L4, SuperstructureState.L4_EJECT),
            Pair.of(SuperstructureState.NET_SCORE, SuperstructureState.NET_SCORE_EJECT));
        for (var pair : shootStates) {
            addEdge(pair.getFirst(), pair.getSecond(), true, false);
        }
        for (var from : statesBelowNoFlip) {
            for (var to : statesBelowNoFlip) {
                if (from != to) {
                    addEdge(from, to, false);
                }
            }
        }
        for (var from : statesAboveFlip) {
            for (var to : statesAboveFlip) {
                if (from != to) {
                    addEdge(from, to, false);
                }
            }
        }
        for (var from : statesBelowFlip){
            for (var to : statesBelowFlip){
                if (from != to) {
                    addEdge(from, to, false);
                }
            }
        }
        for (var from : statesBelowFlip){
            for (var to : statesAboveFlip){
                if (from != to) {
                    addEdge(from, to, true, false);
                }
            }
        }
        for (var from : statesAboveFlip){
            addEdge(from, SuperstructureState.AVOID, true, false);
        }
        for (var from : statesBelowFlip) {
            addEdge(from, SuperstructureState.AVOID, true, false);
        }
        for (var from : statesBelowNoFlip){
            addEdge(from, SuperstructureState.AVOID, true, false);
        }

        setDefaultCommand(runGoal(() -> SuperstructureState.IDLE));
    }
    final Set<SuperstructureState> statesBelowNoFlip =
        Set.of(
            SuperstructureState.CORAL_GROUND_INTAKE,
            SuperstructureState.L1_INTAKE_SIDE,
            SuperstructureState.IDLE
            );
    final Set<SuperstructureState> statesAboveFlip =
        Set.of(
            SuperstructureState.CORAL_STOW,
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.L3,
            SuperstructureState.L4,
            SuperstructureState.P2,
            SuperstructureState.NET_SCORE
            );
    final Set<SuperstructureState> statesBelowFlip =
        Set.of(
            SuperstructureState.L1_SHOOT_SIDE,
            SuperstructureState.L2,
            SuperstructureState.P1
            );


    @Override
    public void periodic() {
        // Run periodic
        intake.periodic();
        endEffectorArm.periodic();
        elevator.periodic();

        currentPoseVisualizer.update(
            elevator.getElevatorPosition(),
            intake.getCurrentAngle(), 
            endEffectorArm.getCurrentAngle()
        );
        setpointPoseVisualizer.update(
            elevator.getWantedPosition(),
            intake.getWantedAngle(), 
            endEffectorArm.getWantedAngle()
        );
        goalPoseVisualizer.update(
            goal.getValue().getPose().elevatorHeight().getAsDouble(),
            goal.getValue().getPose().intakeAngle().getAsDouble(),
            goal.getValue().getPose().endEffectorAngle().getAsDouble()
        );
        if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
            // Update edge to new state
            if (next != null) {
                state = next;
                next = null;
            }

            // Schedule next command in sequence
            if (state != goal) {
                bfs(state, goal)
                    .ifPresent(next -> {
                        this.next = next;
                        edgeCommand = graph.getEdge(state, next);
                        edgeCommand.getCommand().schedule();
                    });
            }
        }

        // Log state
        Logger.recordOutput("Superstructure/State", state);
        Logger.recordOutput("Superstructure/Next", next);
        Logger.recordOutput("Superstructure/Goal", goal);
        if (edgeCommand != null) {
            Logger.recordOutput(
                "Superstructure/EdgeCommand",
                graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
        } else {
            Logger.recordOutput("Superstructure/EdgeCommand", "");
        }
    }

    @AutoLogOutput(key = "Superstructure/AtGoal")
    public boolean atGoal() {
      return state == goal;
    }
  
    
    public Command runGoal(SuperstructureState goal) {
        return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
    }

    public Command runGoal(Supplier<SuperstructureState> goal) {
        return run(() -> setGoal(goal.get()));
    }


    public void setGoal(SuperstructureState goal) {
        // Don't do anything if goal is the same
        if (this.goal == goal) return;
        this.goal = goal;

        if (next == null) return;

        var edgeToCurrentState = graph.getEdge(next, state);
        // Figure out if we should schedule a different command to get to goal faster
        if (edgeCommand.getCommand().isScheduled()
            && edgeToCurrentState != null
            && isEdgeAllowed(edgeToCurrentState, goal)) {
            // Figure out where we would have gone from the previous state
            bfs(state, goal)
                .ifPresent(newNext -> {
                    if (newNext == next) {
                        // We are already on track
                        return;
                    }

                    if (newNext != state && graph.getEdge(next, newNext) != null) {
                        // We can skip directly to the newNext edge
                        edgeCommand.getCommand().cancel();
                        edgeCommand = graph.getEdge(state, newNext);
                        edgeCommand.getCommand().schedule();
                        next = newNext;
                    } else {
                        // Follow the reverse edge from next back to the current edge
                        edgeCommand.getCommand().cancel();
                        edgeCommand = graph.getEdge(next, state);
                        edgeCommand.getCommand().schedule();
                        var temp = state;
                        state = next;
                        next = temp;
                    }
                });
        }
    }

    private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
        // Map to track the parent of each visited node
        Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
        Queue<SuperstructureState> queue = new LinkedList<>();
        queue.add(start);
        parents.put(start, null); // Mark the start node as visited with no parent

        // Perform BFS
        while (!queue.isEmpty()) {
            SuperstructureState current = queue.poll();
            // Check if we've reached the goal
            if (current == goal) {
                break;
            }
            // Process valid neighbors
            for (EdgeCommand edge : graph.outgoingEdgesOf(current)) {
                SuperstructureState neighbor = graph.getEdgeTarget(edge);
                // Only process unvisited neighbors
                if (!parents.containsKey(neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        // Reconstruct the path to the goal if found
        if (!parents.containsKey(goal)) {
            return Optional.empty(); // Goal not reachable
        }

        // Trace back the path from goal to start
        SuperstructureState nextState = goal;
        while (!nextState.equals(start)) {
            SuperstructureState parent = parents.get(nextState);
            if (parent == null) {
                return Optional.empty(); // No valid path found
            } else if (parent.equals(start)) {
                // Return the edge from start to the next node
                return Optional.of(nextState);
            }
            nextState = parent;
        }
        return Optional.of(nextState);
    }

    private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
        return !edge.isRestricted() || goal == graph.getEdgeTarget(edge); 
    }

    /**
     * Adds an edge between two states in the superstructure state machine.
     * @param from The source state
     * @param to The target state
     * @param reverse If true, also adds a reverse edge from 'to' to 'from'
     * @param restricted If true, this edge can only be used when transitioning directly to its target state(z.b from L4 to L4shoot))
     * @see #isEdgeAllowed(EdgeCommand, SuperstructureState) Implementation of restricted edge behavior
     * @see #bfs(SuperstructureState, SuperstructureState) Path finding that respects restricted edges
     */
    private void addEdge(
        SuperstructureState from,
        SuperstructureState to,
        boolean reverse,
        boolean restricted) {
      graph.addEdge(
          from,
          to,
          EdgeCommand.builder()
              .command(getEdgeCommand(from, to))
              .restricted(restricted)
              .build());
      if (reverse) {
        graph.addEdge(
            to,
            from,
            EdgeCommand.builder()
                .command(getEdgeCommand(to, from))
                .restricted(restricted)
                .build());
      }
    }

    /**
     * Adds a non-reversible edge between two states.
     * @param from The source state
     * @param to The target state
     * @param restricted If true, this edge can only be used when transitioning directly to its target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to, boolean restricted) {
        addEdge(from, to, false, restricted);
    }

    /**
     * Adds a non-reversible, non-restricted edge between two states.
     * @param from The source state
     * @param to The target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to) {
        addEdge(from, to, false);
    }



    /** All edge commands should finish and exit properly. */
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
      private final Command command;
      @Builder.Default private final boolean restricted = false;
    }

    private Command runIntake(DoubleSupplier pivotAngle) {
        return Commands.runOnce(() ->intake.setPivotAngle(pivotAngle));
    }

    private Command runEndEffectorArm(DoubleSupplier pivotAngle) {
        return Commands.runOnce(() ->endEffectorArm.setPivotAngle(pivotAngle));
    }

    private Command runElevator(DoubleSupplier position) {
        return Commands.runOnce(() ->elevator.setElevatorPosition(position));
    }
    
      /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
    private Command runSuperstructurePose(SuperstructurePose pose) {
        return runElevator(pose.elevatorHeight())
            .alongWith(runEndEffectorArm(pose.endEffectorAngle())
            .alongWith(runIntake(pose.intakeAngle())));
    }
    private boolean poseAtGoal(){
        return elevator.isAtGoal() && endEffectorArm.isAtGoal() && intake.isAtGoal();
    }
    private Command runSuperstructureRollers(SuperstructureState state){
        return Commands.runOnce(() ->{
            endEffectorArm.setRollerVoltage(state.getValue().getEndEffectorVolts());
            intake.setRollerVoltage(state.getValue().getIntakeVolts());
        });
    }

    //declare all edge commands here
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        if (to == SuperstructureState.AVOID) {
            if (statesBelowFlip.contains(from)) {
                return runElevator(to.getValue().getPose().elevatorHeight())
                    .andThen(
                        Commands.waitUntil(elevator::isAtGoal),
                        runEndEffectorArm(to.getValue().getPose().endEffectorAngle()),
                        runIntake(to.getValue().getPose().intakeAngle()),
                        Commands.waitUntil(this::poseAtGoal)
                    );
            } else if (statesAboveFlip.contains(from)) {
                return runSuperstructurePose(to.getValue().getPose())
                    .andThen(Commands.waitUntil(endEffectorArm::isAtGoal));
            } else {
                return runSuperstructurePose(to.getValue().getPose())
                    .andThen(Commands.waitUntil(elevator::isAtGoal));
            }
        }
        return runSuperstructurePose(to.getValue().getPose())
            .alongWith(runSuperstructureRollers(to))
            .andThen(Commands.waitUntil(this::poseAtGoal));
    }
} 