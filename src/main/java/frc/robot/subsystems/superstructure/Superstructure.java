package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import frc.robot.utils.LoggedTracer;
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

    @Getter
    private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter
    private SuperstructureState goal = SuperstructureState.START;
    private EdgeCommand edgeCommand;
    private final IntakeSubsystem intake;
    private final EndEffectorArmSubsystem endEffectorArm;
    private final ElevatorSubsystem elevator;
    private final SuperstructureVisualizer measuredPoseVisualizer;
    private final SuperstructureVisualizer setpointPoseVisualizer;
    private final SuperstructureVisualizer goalPoseVisualizer;
    private final Set<Pair<SuperstructureState, SuperstructureState>> shootStates;

    /**
     * Constructor for the Superstructure subsystem.
     * <p>
     * See README.md for detailed implementation requirements and framework documentation.
     *
     * @see #addEdge(SuperstructureState, SuperstructureState)
     * @see #getEdgeCommand(SuperstructureState, SuperstructureState)
     * @see #setDefaultCommand(Command)
     * @see EdgeCommand
     */
    public Superstructure(IntakeSubsystem intake, EndEffectorArmSubsystem endEffectorArm, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.endEffectorArm = endEffectorArm;
        this.elevator = elevator;
        this.measuredPoseVisualizer = new SuperstructureVisualizer("Measured");
        this.setpointPoseVisualizer = new SuperstructureVisualizer("Setpoint");
        this.goalPoseVisualizer = new SuperstructureVisualizer("Goal");

        // Add states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }

        // Declear all edges here
        addEdge(SuperstructureState.START, SuperstructureState.IDLE, false, false);
        // Add edges between shoot and preshoot states
        this.shootStates =
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
        for (var from : statesBelowFlip) {
            for (var to : statesBelowFlip) {
                if (from != to) {
                    addEdge(from, to, false);
                }
            }
        }
        for (var from : statesBelowFlip) {
            for (var to : statesAboveFlip) {
                if (from != to) {
                    addEdge(from, to, true, false);
                }
            }
        }
        for (var from : statesAboveFlip) {
            addEdge(from, SuperstructureState.AVOID, true, false);
        }
        for (var from : statesBelowFlip) {
            addEdge(from, SuperstructureState.AVOID, true, false);
        }
        for (var from : statesBelowNoFlip) {
            addEdge(from, SuperstructureState.AVOID, true, false);
        }


        setDefaultCommand(
                runGoal(() -> {
                    if (endEffectorArm.isHasCoral()) {
                        return SuperstructureState.CORAL_STOW;
                    }
                    if (endEffectorArm.isHasAlgae()) {
                        return SuperstructureState.ALGAE_STOW;
                    }
                    if (AimGoalSupplier.isInHexagonalReefDangerZone(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d())) {
                        return SuperstructureState.AVOID;
                    }
                    if (intake.isIndexRollerHasCoral()) {
                        return SuperstructureState.CORAL_GROUND_INTAKE;
                    }
                    
                    return SuperstructureState.IDLE;
             
                })
        );
    }

    final Set<SuperstructureState> statesBelowNoFlip =
            Set.of(
                    SuperstructureState.CORAL_GROUND_INTAKE,
                    SuperstructureState.L1_INTAKE_SIDE,
                    SuperstructureState.IDLE,
                    SuperstructureState.CORAL_OUTTAKE
            );
    final Set<SuperstructureState> statesAboveFlip =
            Set.of(
                    SuperstructureState.CORAL_STOW,
                    SuperstructureState.ALGAE_STOW,
                    SuperstructureState.CORAL_INDEXED_INTAKE,
                    SuperstructureState.L3,
                    SuperstructureState.L4,
                    SuperstructureState.P2,
                    SuperstructureState.NET_SCORE
            );
    final Set<SuperstructureState> statesBelowFlip =
            Set.of(
                    SuperstructureState.L1_SHOOT_SIDE,
                    SuperstructureState.L2,
                    SuperstructureState.P1,
                    SuperstructureState.CORAL_STATION_INTAKE
            );


    @Override
    public void periodic() {
        // Run periodic
        intake.periodic();
        endEffectorArm.periodic();
//        elevator.periodic();

        //simulated gamepiece tracking
        if (!RobotBase.isReal() && !RobotConstants.useReplay) {
            if (atGoal() && state == SuperstructureState.CORAL_GROUND_INTAKE) {
                intake.setIndexRollerHasCoral(false);
                endEffectorArm.setHasCoral(true);
            }
            if (atGoal() && state == SuperstructureState.CORAL_INDEXED_INTAKE) {
                intake.setIndexRollerHasCoral(true);
                // Keep algae state unchanged since we're just indexing coral
            }
            for (var pair : shootStates) {
                if (atGoal() && state == pair.getSecond()) {
                    endEffectorArm.setHasCoral(false);
                }
            }
            if (atGoal() && (state == SuperstructureState.P1 || state == SuperstructureState.P2)) {
                endEffectorArm.setHasAlgae(true);
            }
            if (atGoal() && state == SuperstructureState.NET_SCORE_EJECT) {
                endEffectorArm.setHasAlgae(false);
            }
        }

        //log the gamepiece tracking
        measuredPoseVisualizer.logCoralPose3D(
                intake.isIndexRollerHasCoral(),
                endEffectorArm.isHasCoral(),
                endEffectorArm.isHasAlgae()
        );

        measuredPoseVisualizer.update(
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
        // if we complete the current command, there are three things that we should planning to do
        // 1. update state (to next, which is current state)
        // 2. update next (to the next target state, which should be found through bfs)
        // 3. update edgeCommand (the command we should process to reach next)
        if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
            // why we need this if statement ?
            // from my perspective, if next is null in this case, we should abort the process
            // this part complete 1.
            if (next != null) {
                // change the old state to current state since we already complete the command
                state = next;
                // change next state to null temporarily
                next = null;
            }

            // now we should complete 2 and 3 by using bfs
            // Schedule next command in sequence
            if (state != goal) {
                // the bfs will find the shortest path between our current state and the goal and
                // return the first point(state) to get to the goal
                bfs(state, goal)
                        .ifPresent(next -> {
                            // this setup the next state
                            this.next = next;
                            // find the edge(command) between state and next
                            edgeCommand = graph.getEdge(state, next);
                            // schedule the command to get to the next state
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

        // Record cycle time
        LoggedTracer.record("Superstructure");
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

    public Command runZero() {
        return runGoal(() -> SuperstructureState.IDLE)
                .until(this::poseAtGoal)
                .andThen(elevator.zeroElevator());
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
     *
     * @param from       The source state
     * @param to         The target state
     * @param reverse    If true, also adds a reverse edge from 'to' to 'from'
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
     *
     * @param from       The source state
     * @param to         The target state
     * @param restricted If true, this edge can only be used when transitioning directly to its target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to, boolean restricted) {
        addEdge(from, to, false, restricted);
    }

    /**
     * Adds a non-reversible, non-restricted edge between two states.
     *
     * @param from The source state
     * @param to   The target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to) {
        addEdge(from, to, false);
    }


    /**
     * All edge commands should finish and exit properly.
     */
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
        private final Command command;
        @Builder.Default
        private final boolean restricted = false;
    }

    public boolean hasCoral() {
        return endEffectorArm.isHasCoral();
    }

    public boolean hasAlgae() {
        return endEffectorArm.isHasAlgae();
    }

    public boolean indexedCoral() {
        return intake.isIndexRollerHasCoral();
    }

    public double getElevatorPosition() {
        return elevator.getElevatorPosition();
    }

    private Command runIntake(DoubleSupplier pivotAngle) {
        return Commands.runOnce(() -> intake.setPivotAngle(pivotAngle));
    }

    private Command runEndEffectorArm(DoubleSupplier pivotAngle) {
        return Commands.runOnce(() -> endEffectorArm.setPivotAngle(pivotAngle));
    }

    private Command runElevator(DoubleSupplier position) {
        return Commands.runOnce(() -> elevator.setElevatorPosition(position));
    }

    /**
     * Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately.
     */
    private Command runSuperstructurePose(SuperstructurePose pose) {
        return runElevator(pose.elevatorHeight())
                .alongWith(runEndEffectorArm(pose.endEffectorAngle())
                        .alongWith(runIntake(pose.intakeAngle())));
    }

    public boolean poseAtGoal() {
        return elevator.isAtGoal() && endEffectorArm.isAtGoal() && intake.isAtGoal();
    }

    private Command runSuperstructureRollers(SuperstructureState state) {
        return Commands.runOnce(() -> {
            endEffectorArm.setRollerVoltage(state.getValue().getEndEffectorVolts());
            intake.setIntakeRollerVoltage(state.getValue().getIntakeVolts());
            intake.setIndexRollerVoltage(state.getValue().getIndexRollerVolts());
        });
    }

    // declare all edge commands here
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        if (from == SuperstructureState.START && to == SuperstructureState.IDLE) {
            return runEndEffectorArm(to.getValue().getPose().endEffectorAngle())
                    .alongWith(runIntake(to.getValue().getPose().intakeAngle()))
                    .andThen(
                            Commands.waitUntil(endEffectorArm::isAtGoal),
                            elevator.zeroElevator(),
                            Commands.waitUntil(() -> !elevator.isZeroing())
                    );
        }
        if (from == SuperstructureState.CORAL_GROUND_INTAKE || from == SuperstructureState.CORAL_OUTTAKE) {
            return runSuperstructurePose(to.getValue().getPose())
                    .alongWith(runSuperstructureRollers(to))
                    .andThen(Commands.waitUntil(this::poseAtGoal));
        }
        // Special handling for coral indexing while holding algae - only move intake
        if (to == SuperstructureState.CORAL_INDEXED_INTAKE) {
            return runIntake(to.getValue().getPose().intakeAngle())
                    .andThen(
                            Commands.waitUntil(intake::isAtGoal),
                            runSuperstructureRollers(to)
                    )
                    .andThen(Commands.waitUntil(this::poseAtGoal));
        
        }
        if (to == SuperstructureState.L4){
            return runElevator(to.getValue().getPose().elevatorHeight())
                .andThen(
                    Commands.waitUntil(elevator::isAtGoal),
                    runSuperstructurePose(to.getValue().getPose()),
                    Commands.waitUntil(this::poseAtGoal));
        }
        if (to == SuperstructureState.CORAL_STATION_INTAKE){
            return runEndEffectorArm(to.getValue().getPose().endEffectorAngle())
                    .andThen(Commands.waitUntil(endEffectorArm::isAtGoal),
                                runSuperstructurePose(to.getValue().getPose()),
                                Commands.waitUntil(this::poseAtGoal),
                                runSuperstructureRollers(to));
        }
        // is safe to flip inorder to produce a smoother elevator motion
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
                //TODO:check this 
                return runSuperstructurePose(to.getValue().getPose())
                        .andThen(Commands.waitUntil(elevator::isSafeToFlip));
            }
        }
        return runSuperstructurePose(to.getValue().getPose())
                .andThen(Commands.waitUntil(this::poseAtGoal).andThen(runSuperstructureRollers(to)));
    }
} 