package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {


    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    @Getter private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter private SuperstructureState goal = SuperstructureState.START;
    private EdgeCommand edgeCommand;

    public Superstructure() {


        // Add states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }

        // Declear all edges here
        addEdge(SuperstructureState.START, SuperstructureState.STOW);
        addEdge(SuperstructureState.CORAL_GROUND_INTAKE, SuperstructureState.STOW, true, false);
        addEdge(SuperstructureState.CORAL_GROUND_INTAKE, SuperstructureState.L3, true, false);
        addEdge(SuperstructureState.STOW, SuperstructureState.L3, true, false);
        addEdge(SuperstructureState.L3, SuperstructureState.L3_EJECT, true,false);
    }

    @Override
    public void periodic() {
        // Run periodic
   
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

    //declare all edge commands here
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        if (from == SuperstructureState.CORAL_GROUND_INTAKE && to == SuperstructureState.L3) {
            return Commands.sequence(
                Commands.waitSeconds(1),
                Commands.runOnce(() -> {
                    System.out.println("Coral Ground Intake to L3");
                })
            );
        }
        return Commands.runOnce(() -> {
            System.out.println(from + " to " + to);
        });
    }

    /** All edge commands should finish and exit properly. */
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
      private final Command command;
      @Builder.Default private final boolean restricted = false;
    }
} 