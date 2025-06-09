package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.superstructure.SuperstructureStateData.SuperstructureStateDataBuilder;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    // Stow positions
    START(createState(Preset.START, null, null)),
    CORAL_STOW(createState(Preset.CORAL_STOW, null, null)),
    ALGAE_STOW(createState(Preset.ALGAE_STOW, null, null)),
    IDLE(createState(Preset.IDLE, null, null)),
    AVOID(createState(Preset.AVOID, null, null)),

    // L1 positions
    L1_INTAKE_SIDE(createState(Preset.L1_INTAKE_SIDE, null, null)),
    L1_INTAKE_SIDE_EJECT(createState(L1_INTAKE_SIDE, () -> IntakeConstants.OUTTAKE_VOLTAGE.get(), null)),
    L1_SHOOT_SIDE(createState(Preset.L1_SHOOT_SIDE, null, null)),
    L1_SHOOT_SIDE_EJECT(createState(L1_SHOOT_SIDE, null, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE_L1.get())),

    // L2-L4 positions
    L2(createState(Preset.L2, null, null)),
    L2_EJECT(createState(L2, null, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L3(createState(Preset.L3, null, null)),
    L3_EJECT(createState(L3, null, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L4(createState(Preset.L4, null, null)),
    L4_EJECT(createState(L4, null, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),

    // Net scoring positions
    NET_SCORE(createState(Preset.NET_SCORE, null, null)),
    NET_SCORE_EJECT(createState(NET_SCORE, null, () -> EndEffectorArmConstants.ALGAE_NET_SHOOT_VOLTAGE.get())),

    // Processor scoring positions
    PROCESSOR_SCORE(createState(Preset.PROCESSOR, null, null)),
    PROCESSOR_SCORE_EJECT(createState(PROCESSOR_SCORE, null, () -> EndEffectorArmConstants.ALGAE_PROCESSOR_SHOOT_VOLTAGE.get())),

    // Pickup positions
    P1(createState(Preset.P1, null, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    P2(createState(Preset.P2, null, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    CORAL_GROUND_INTAKE(createState(Preset.CORAL_GROUND_INTAKE, () -> IntakeConstants.INTAKE_VOLTAGE.get(), () -> EndEffectorArmConstants.CORAL_INTAKE_VOLTAGE.get()));

    private final SuperstructureStateData value;

    // Helper methods to create states
    private static SuperstructureStateData createState(Preset preset, DoubleSupplier intakeVoltage, DoubleSupplier eeVoltage) {
        var builder = SuperstructureStateData.builder()
            .pose(preset.getPose());
        
        if (intakeVoltage != null) {
            builder.intakeVolts(intakeVoltage);
        }
        if (eeVoltage != null) {
            builder.endEffectorVolts(eeVoltage);
        }
        
        return builder.build();
    }

    private static SuperstructureStateData createState(SuperstructureState baseState, DoubleSupplier intakeVoltage, DoubleSupplier eeVoltage) {
        var builder = baseState.getValue().toBuilder();
        
        if (intakeVoltage != null) {
            builder.intakeVolts(intakeVoltage);
        }
        if (eeVoltage != null) {
            builder.endEffectorVolts(eeVoltage);
        }
        
        return builder.build();
    }
}