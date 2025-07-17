package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;
import java.util.function.DoubleSupplier;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    // Stow positions
    START(createState(Preset.START)),
    AUTO_START(createState(Preset.AUTO_START)),
    CORAL_STOW(createEEState(Preset.CORAL_STOW, () -> EndEffectorArmConstants.CORAL_HOLD_VOLTAGE.get())),
    ALGAE_STOW(createEEState(Preset.ALGAE_STOW, () -> EndEffectorArmConstants.ALGAE_HOLD_VOLTAGE.get())),
    IDLE(createState(Preset.IDLE)),
    AVOID(createState(Preset.AVOID)),

    // L1 positions
    //TODO: L1_intake_side
    L1_INTAKE_SIDE(createState(Preset.L1_INTAKE_SIDE)),
    L1_INTAKE_SIDE_EJECT(createIntakeState(L1_INTAKE_SIDE, () -> IntakeConstants.SHOOT_VOLTAGE.get())),
    L1_INTAKE_SIDE_DOWN(createEEState(Preset.L1_INTAKE_SIDE_DOWN, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE_L1.get())),
    L1_SHOOT_SIDE(createState(Preset.L1_SHOOT_SIDE)),
    L1_SHOOT_SIDE_EJECT(createEEState(L1_SHOOT_SIDE, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE_L1.get())),

    // L2-L4 positions
    L2(createState(Preset.L2)),
    L2_EJECT(createEEState(L2, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L3(createState(Preset.L3)),
    L3_EJECT(createEEState(L3, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L4(createState(Preset.L4)),
    L4_EJECT(createEEState(L4, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),

    // L1 positions
    L1(createState(Preset.L1_SHOOT_SIDE)),

    // Net scoring positions
    NET_SCORE(createState(Preset.NET_SCORE)),
    NET_SCORE_EJECT(createEEState(NET_SCORE, () -> EndEffectorArmConstants.ALGAE_NET_SHOOT_VOLTAGE.get())),

    // Processor scoring positions
    PROCESSOR_SCORE(createState(Preset.PROCESSOR)),
    PROCESSOR_SCORE_EJECT(createEEState(PROCESSOR_SCORE, () -> EndEffectorArmConstants.ALGAE_PROCESSOR_SHOOT_VOLTAGE.get())),

    // Pickup positions
    P1(createEEState(Preset.P1, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    P2(createEEState(Preset.P2, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    CORAL_GROUND_INTAKE(createState(Preset.CORAL_GROUND_INTAKE, 
        () -> IntakeConstants.INTAKE_VOLTAGE.get(), 
        () -> IntakeConstants.INDEX_ROLLER_VOLTAGE.get(),
        () -> EndEffectorArmConstants.CORAL_INTAKE_VOLTAGE.get())),
    CORAL_OUTTAKE(createState(Preset.CORAL_GROUND_INTAKE, ()-> IntakeConstants.OUTTAKE_VOLTAGE.get(),() -> -IntakeConstants.INDEX_ROLLER_VOLTAGE.get(),
    () -> EndEffectorArmConstants.CORAL_OUTTAKE_VOLTAGE.get())),
    CORAL_INDEXED_INTAKE(createState(Preset.CORAL_INDEXED_INTAKE,
        () -> IntakeConstants.INTAKE_HOLD_VOLTAGE.get(), 
        () -> -IntakeConstants.INDEX_ROLLER_VOLTAGE.get(),
        () -> EndEffectorArmConstants.ALGAE_HOLD_VOLTAGE.get())),
    CORAL_STATION_INTAKE(createState(Preset.CORAL_STATION_INTAKE,
        () -> 0, 
        () -> 0,
        () -> EndEffectorArmConstants.CORAL_INTAKE_VOLTAGE.get())),
    
    CORAL_L1_INTAKE(createState(Preset.CORAL_L1_INTAKE,
        () -> IntakeConstants.INTAKE_HOLD_VOLTAGE.get(), 
        () -> -IntakeConstants.INDEX_ROLLER_VOLTAGE.get(),
        () -> EndEffectorArmConstants.CORAL_OUTTAKE_VOLTAGE.get())),

    // Safe outtake positions
    SAFE_OUTTAKE(createState(
        Preset.SAFE_OUTTAKE,
        () -> IntakeConstants.OUTTAKE_VOLTAGE.get(),
        () -> -IntakeConstants.INDEX_ROLLER_VOLTAGE.get(),
        () -> EndEffectorArmConstants.CORAL_OUTTAKE_VOLTAGE.get()));


    private final SuperstructureStateData value;

    // Creates a basic state with just the pose
    private static SuperstructureStateData createState(Preset preset) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .build();
    }

    // Creates a state with end effector voltage
    private static SuperstructureStateData createEEState(Preset preset, DoubleSupplier eeVoltage) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .endEffectorVolts(eeVoltage)
            .build();
    }

    // Creates a state with end effector voltage from an existing state
    private static SuperstructureStateData createEEState(SuperstructureState baseState, DoubleSupplier eeVoltage) {
        return baseState.getValue().toBuilder()
            .endEffectorVolts(eeVoltage)
            .build();
    }

    // Creates a state with intake voltage
    private static SuperstructureStateData createIntakeState(SuperstructureState baseState, DoubleSupplier intakeVoltage) {
        return baseState.getValue().toBuilder()
            .intakeVolts(intakeVoltage)
            .build();
    }

    // Creates a state with both intake and end effector voltages
    private static SuperstructureStateData createState(Preset preset, DoubleSupplier intakeVoltage, DoubleSupplier indexRollerVoltage, DoubleSupplier eeVoltage) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .intakeVolts(intakeVoltage)
            .indexRollerVolts(indexRollerVoltage)
            .endEffectorVolts(eeVoltage)
            .build();
    }
}