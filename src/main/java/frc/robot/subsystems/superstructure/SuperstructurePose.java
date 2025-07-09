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
public record SuperstructurePose(DoubleSupplier elevatorHeight, DoubleSupplier endEffectorAngle, DoubleSupplier intakeAngle) {
    /**
     * Preset poses for the superstructure.
     * Provides two ways to create poses:
     * 1. Dynamic poses using suppliers for runtime-calculated positions
     * 2. Static poses using fixed values that can be tuned through NetworkTables
     */
    @Getter
    @RequiredArgsConstructor
    enum Preset {
        CORAL_STOW("Coral Stow",        0.6,   135,   60),
        ALGAE_STOW("Algae Stow",        0.6,    0,   60),
        START("Start",                  0.5,     0,    60),
        L1_INTAKE_SIDE("L1 Intake Side", 0.16,   0,   60),
        L1_SHOOT_SIDE("L1 Shoot Side",  0.3,   200,   60),
        L2("L2",                        0.16,  190,   60),
        L3("L3",                        0.56, 190,   60),
        L4("L4",                        1.43,   220,   60),
        NET_SCORE("Net Score",          1.4,   -120,   60),
        P1("P1",                        0.52,    -10,   60),
        P2("P2",                        0.85,    -10,   60),
        CORAL_GROUND_INTAKE("Coral Ground Intake", 0.01, 0, 142),
        CORAL_INDEXED_INTAKE("Coral Indexed Intake", 0.57, 0, 142),
        CORAL_STATION_INTAKE("Coral Station Intake", 0.2, 180, 60),
        IDLE("Idle",                    0.01,    0,   132),
        PROCESSOR("Processor", 0.16, 0, 60),
        AVOID("Avoid",                  0.6,     0,   60);


        
        private final SuperstructurePose pose;

        Preset(DoubleSupplier elevatorHeight, DoubleSupplier endEffectorAngle, DoubleSupplier intakeAngle) {
            this(
                new SuperstructurePose(
                    elevatorHeight, endEffectorAngle, intakeAngle));
          }
      
        Preset(String name, double elevatorHeight, double endEffectorAngle, double intakeAngle) {
            this(
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
                new LoggedTunableNumber("Superstructure/" + name + "/Pivot", endEffectorAngle),
                new LoggedTunableNumber("Superstructure/" + name + "/Intake", intakeAngle));
        }
    }
} 