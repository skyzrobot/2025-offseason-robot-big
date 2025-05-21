package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.LoggedTunableNumber;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Represents a pose of the superstructure with suppliers for dynamic position control.
 * Uses suppliers to enable both dynamic and static positions.
 */
public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> endEffectorAngle, Supplier<Rotation2d> intakeAngle) {
    /**
     * Preset poses for the superstructure.
     * Provides two ways to create poses:
     * 1. Dynamic poses using suppliers for runtime-calculated positions
     * 2. Static poses using fixed values that can be tuned through NetworkTables
     */
    @Getter
    @RequiredArgsConstructor
    enum Preset {
        STOW("stow",0.5, 45, 0),
        START("start", 0.0, 0, 0),
        L3("L3",0.7,45,45),
        CORAL_GROUND_INTAKE("Coral Ground Intake", 0.0, 0, 90);
        
        private final SuperstructurePose pose;

        Preset(DoubleSupplier elevatorHeight, DoubleSupplier endEffectorAngle, DoubleSupplier intakeAngle) {
            this(
                new SuperstructurePose(
                    elevatorHeight, () -> Rotation2d.fromDegrees(endEffectorAngle.getAsDouble()), () -> Rotation2d.fromDegrees(intakeAngle.getAsDouble())));
          }
      
        Preset(String name, double elevatorHeight, double endEffectorAngle, double intakeAngle) {
            this(
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
                new LoggedTunableNumber("Superstructure/" + name + "/Pivot", endEffectorAngle),
                new LoggedTunableNumber("Superstructure/" + name + "/Intake", intakeAngle));
        }

        
    }
} 