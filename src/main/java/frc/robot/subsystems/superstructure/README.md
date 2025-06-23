# Superstructure Framework

## Overview

The Superstructure is a state machine-based framework for coordinating multiple robot subsystems into a cohesive, intelligent system. It manages complex multi-subsystem movements and ensures safe, efficient transitions between different robot configurations.

## Design Philosophy

### Why a State Machine Approach?

1. **Coordination**: Multiple subsystems (elevator, intake, end effector) must work together safely
2. **Safety**: Prevents dangerous configurations (e.g., extending elevator while arm is in wrong position)
3. **Efficiency**: Automatically finds shortest path between any two states
4. **Maintainability**: Clear, declarative state definitions make the system easy to understand and modify
5. **Debugging**: State transitions are logged and visible, making issues easier to diagnose

### Key Design Principles

- **Declarative States**: Each robot configuration is defined as a state with clear pose and action parameters
- **Graph-Based Transitions**: States are connected by edges representing safe transition paths
- **Automatic Path Planning**: BFS algorithm finds optimal routes between states
- **Hardware Abstraction**: IO interfaces enable simulation and easy hardware swapping
- **Command-Based Integration**: Seamlessly integrates with WPILib's command-based framework

## Architecture Components

### Core Classes

1. **`Superstructure.java`** - Main state machine controller
2. **`SuperstructureState.java`** - Enum defining all possible robot states
3. **`SuperstructurePose.java`** - Record defining physical positions for each subsystem
4. **`SuperstructureStateData.java`** - Data container for states including poses and voltages
5. **`SuperstructureVisualizer.java`** - 3D visualization and logging support

### State Machine Flow

```
[Current State] --[EdgeCommand]--> [Next State] --[EdgeCommand]--> [Goal State]
```

1. **State Transition**: When goal is set, BFS finds shortest path
2. **Edge Execution**: Commands execute to move between adjacent states
3. **Safety Validation**: Restricted edges prevent unsafe direct transitions
4. **Automatic Progression**: System automatically continues until goal is reached

## Implementation Requirements

### Critical Requirements

1. **All state transitions (edges) must be declared in the constructor** using `addEdge(SuperstructureState, SuperstructureState)`
2. **All edge commands must be implemented** in `getEdgeCommand(SuperstructureState, SuperstructureState)`
3. **All commands must properly exit and finish** - This is crucial for the state machine to progress to the next state

### Default Command Behavior

The superstructure uses a default command that continuously monitors and updates its state:
- Runs when robot transitions from disabled to enabled
- Runs when no other command is actively running on the subsystem  
- Runs after a previously running command completes

**Important**: When `runGoal(SuperstructureState)` is executed (e.g., via button bindings), it remains active even after reaching the target state. This prevents the default command from taking over, maintaining the desired state until a new command is issued.

## Subsystem Selection Guidelines

### Which Subsystems Should Be Included

**Include subsystems that:**
1. **Move together coordinately** - Multiple subsystems that must work together to achieve robot goals
2. **Have interdependent poses** - Subsystems where one's position affects another's safe operation
3. **Share game pieces** - Subsystems that transfer objects between each other
4. **Require coordinated safety** - Subsystems that could collide or interfere with each other
5. **Form logical groupings** - Subsystems that represent a cohesive "arm" or "mechanism"

**Examples from this implementation:**
- ✅ **Elevator + EndEffectorArm + Intake** - Work together to pick up, move, and score game pieces
- ✅ **Coordinated movement** - Elevator must be at safe height before arm extends
- ✅ **Game piece transfer** - Coral moves from intake to end effector
- ✅ **Safety dependencies** - Arm position affects elevator safe travel

### Which Subsystems Should Be Excluded

**Exclude subsystems that:**
1. **Operate independently** - Can function without coordination with other subsystems
2. **Have simple control logic** - Single motor movements with basic commands
3. **Don't share physical space** - No collision or interference concerns
4. **Single-purpose operations** - Perform one isolated task without multi-subsystem coordination
5. **Don't require sequencing** - No complex timing or ordering requirements, Unless its safetly related sequencing

**Examples from this implementation:**
- ❌ **Climber** - Only used during endgame, operates independently, simple extend/retract logic
- ❌ **Swerve Drive** - Independent movement system, managed separately by drive commands
- ❌ **Limelight** - Vision processing, doesn't move physically, independent operation
- ❌ **Indicator LEDs** - Status display only, no physical coordination needed

### Action Logic Guidelines

**Include in Superstructure:**
1. **Frequently used robot poses** - Common configurations like stow, scoring positions, pickup positions
2. **Simple state transitions** - Basic movements between well-defined poses
3. **Coordinated positioning** - Multi-subsystem poses that must be reached safely together
4. **Safety-dependent poses** - Configurations where subsystem coordination prevents collisions
5. **Atomic robot configurations** - Individual "states" the robot needs to be in

**Key insight:** The superstructure manages frequently used poses with simple transitions. Complex action sequences are handled by separate commands that orchestrate these state changes.

**Examples from this implementation:**
```java
// ✅ Frequently used poses (IN superstructure)
IDLE, CORAL_STOW, ALGAE_STOW,               // Basic positions
L1_INTAKE_SIDE, L2, L3, L4,                 // Scoring poses
CORAL_GROUND_INTAKE, P1, P2                 // Pickup poses

// ❌ Complex sequences (SEPARATE commands using superstructure)
SuperCycleCommand                           // Uses multiple states over time
ReefAimCommand                              // Continuous aiming with state changes
ClimbCommand                                // Independent subsystem

// ✅ Simple transitions between poses
addEdge(SuperstructureState.IDLE, SuperstructureState.L3);
addEdge(SuperstructureState.L3, SuperstructureState.L3_EJECT, true, false);
```

**Exclude from Superstructure (handle with separate commands):**
1. **Complex action sequences** - Multi-step workflows like SuperCycleCommand
2. **Conditional logic flows** - Commands with complex decision trees and timing
3. **User interaction sequences** - Commands that respond to continuous user input
4. **Multi-state workflows** - Actions that transition through multiple states over time
5. **Game-specific strategies** - High-level autonomous routines and operator sequences



## Building a New Superstructure Framework


### Step 1: Define Your Subsystems

Create your subsystem classes following the Hardware Abstraction pattern:

```java
// Example: ArmSubsystem
public class ArmSubsystem extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;
    }
    
    // Public methods for controlling the arm
    public void setAngle(double angleDegrees) { /* implementation */ }
    public boolean isAtGoal() { /* implementation */ }
    // etc.
}
```

### Step 2: Define Your Poses

Create a `SuperstructurePose` record with suppliers for each subsystem:

```java
public record SuperstructurePose(
    DoubleSupplier armAngle,
    DoubleSupplier elevatorHeight,
    DoubleSupplier wristAngle
) {
    @Getter
    @RequiredArgsConstructor
    enum Preset {
        STOW("Stow", 0, 0.1, 90),
        PICKUP("Pickup", 45, 0.0, 0),
        SCORE_HIGH("Score High", 90, 1.2, 45);
        
        private final SuperstructurePose pose;
        
        Preset(String name, double arm, double elevator, double wrist) {
            this(new SuperstructurePose(
                new LoggedTunableNumber("Superstructure/" + name + "/Arm", arm),
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevator),
                new LoggedTunableNumber("Superstructure/" + name + "/Wrist", wrist)
            ));
        }
    }
}
```

### Step 3: Define Your States

Create a `SuperstructureState` enum with all robot configurations:

```java
@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    START(createState(Preset.STOW)),
    IDLE(createState(Preset.STOW)),
    PICKUP(createState(Preset.PICKUP)),
    SCORE_HIGH(createState(Preset.SCORE_HIGH)),
    SCORE_HIGH_EJECT(createEjectState(SCORE_HIGH, () -> 12.0)); // 12V eject
    
    private final SuperstructureStateData value;
    
    private static SuperstructureStateData createState(Preset preset) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .build();
    }
    
    private static SuperstructureStateData createEjectState(SuperstructureState base, DoubleSupplier voltage) {
        return base.getValue().toBuilder()
            .intakeVolts(voltage)
            .build();
    }
}
```

### Step 4: Update StateData

Modify `SuperstructureStateData` to include voltage suppliers for your subsystems:

```java
@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
    @Builder.Default
    private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> 0.0, () -> 0.0);
    
    @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;
    @Builder.Default private final DoubleSupplier shooterVolts = () -> 0.0;
    // Add suppliers for your subsystems
}
```

### Step 5: Implement Your Superstructure

Create your main `Superstructure` class:

```java
public class Superstructure extends SubsystemBase {
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    
    public Superstructure(ArmSubsystem arm, ElevatorSubsystem elevator, IntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        
        // Add all states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }
        
        // Define your state transitions
        addEdge(SuperstructureState.START, SuperstructureState.IDLE);
        addEdge(SuperstructureState.IDLE, SuperstructureState.PICKUP);
        addEdge(SuperstructureState.PICKUP, SuperstructureState.SCORE_HIGH);
        addEdge(SuperstructureState.SCORE_HIGH, SuperstructureState.SCORE_HIGH_EJECT, true, false); // shoot edge
        
        // Set default command
        setDefaultCommand(runGoal(() -> SuperstructureState.IDLE));
    }
    
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        // Handle special transitions
        if (from == SuperstructureState.START && to == SuperstructureState.IDLE) {
            return runArm(to.getValue().getPose().armAngle())
                .andThen(Commands.waitUntil(arm::isAtGoal))
                .andThen(elevator.zeroElevator());
        }
        
        // Default: move to pose then run rollers
        return runSuperstructurePose(to.getValue().getPose())
            .andThen(Commands.waitUntil(this::poseAtGoal))
            .andThen(runSuperstructureRollers(to));
    }
}
```

### Step 6: Add Safety Considerations

Consider which transitions need special handling:
- **Restricted edges**: Use for shoot commands that should only activate when directly targeting that state
- **Sequential movements**: For safety, move certain subsystems before others
- **Collision avoidance**: Define intermediate states to avoid dangerous configurations

### Step 7: Integration

1. Register your superstructure in `RobotContainer`
2. Bind commands to joystick buttons: `button.onTrue(superstructure.runGoal(SuperstructureState.PICKUP))`
3. Add autonomous commands that use the superstructure states
4. Tune your poses using NetworkTables/Glass

## Debugging Tips

1. **Check Logs**: State transitions are automatically logged
2. **Visualizer**: Use the 3D visualizer to see actual vs. expected positions
3. **Command Completion**: Ensure all edge commands properly finish
4. **BFS Path**: Verify your edge definitions create valid paths between states
5. **Tuning**: Use LoggedTunableNumber for easy pose adjustments

## Common Patterns

### Game Piece Handling
```java
// In default command
if (hasGamePiece()) {
    return SuperstructureState.STOW_WITH_PIECE;
}
return SuperstructureState.IDLE;
```

### Conditional State Transitions
```java
// Only allow shooting when at correct state
addEdge(SuperstructureState.AIM, SuperstructureState.SHOOT, true, true); // restricted
```

### Complex Movements
```java
// Move elevator first for safety
if (needsElevatorFirst(from, to)) {
    return runElevator(to.getPose().elevatorHeight())
        .andThen(Commands.waitUntil(elevator::isAtGoal))
        .andThen(runArm(to.getPose().armAngle()));
}
```

This framework provides a robust, maintainable foundation for complex robot control while maintaining safety and ease of use. 