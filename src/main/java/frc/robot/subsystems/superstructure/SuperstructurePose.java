package frc.robot.subsystems.superstructure;

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
    
    // Static toggle state for intake pose switching
    private static boolean intakeToggleState = false;
    
    /**
     * Sets the intake toggle state for all poses that support alternate intake angles.
     * @param toggled true to use alternate intake angles, false for default angles
     */
    public static void setIntakeToggleState(boolean toggled) {
        intakeToggleState = toggled;
    }
    
    /**
     * Gets the current intake toggle state.
     * @return true if using alternate intake angles, false if using default angles
     */
    public static boolean getIntakeToggleState() {
        return intakeToggleState;
    }

    /**
     * Preset poses for the superstructure.
     * Provides two ways to create poses:
     * 1. Dynamic poses using suppliers for runtime-calculated positions
     * 2. Static poses using fixed values that can be tuned through NetworkTables
     */
    @Getter
    @RequiredArgsConstructor
    enum Preset {
        CORAL_STOW("Coral Stow",        0.53,   135,   132, 60),  // alternate: 110
        ALGAE_STOW("Algae Stow",        0.6,    0,     60,  60),   // alternate: 80
        START("Start",                  0.5,    0,     132),
        L1_INTAKE_SIDE("L1 Intake Side", 0.01,   0,    45),
        L1_SHOOT_SIDE("L1 Shoot Side",  0.3,   200,   132,  60),   // alternate: 40
        L2("L2",                        0.16,  190,   132, 60),  // alternate: 110
        L3("L3",                        0.53,  185,   132, 60),  // alternate: 110
        L4("L4",                        1.445, 225,   132, 60),  // alternate: 110
        NET_SCORE("Net Score",          1.44,   -133,  132,  60),   // alternate: 40
        P1("P1",                        0.52,  -10,   132,60),
        P2("P2",                        0.85,  -10,   132,60),
        CORAL_GROUND_INTAKE("Coral Ground Intake", 0.01, 0, 144),
        CORAL_INDEXED_INTAKE("Coral Indexed Intake", 0.57, 0, 144),
        CORAL_STATION_INTAKE("Coral Station Intake", 0.2, 180, 60),
        CORAL_L1_INTAKE("Coral L1 Intake", 0.01, 0, 144),
        SAFE_OUTTAKE("Safe Outtake", 0.53, 0, 60),
        IDLE("Idle",                    0.01,  0,     132, 60),  // alternate: 110
        PROCESSOR("Processor", 0.16, 0, 60),
        AVOID("Avoid",                  0.51,  0,     132, 60);  // alternate: 110


        
        private final SuperstructurePose pose;

        Preset(DoubleSupplier elevatorHeight, DoubleSupplier endEffectorAngle, DoubleSupplier intakeAngle) {
            this(
                new SuperstructurePose(
                    elevatorHeight, endEffectorAngle, intakeAngle));
          }
        
        // Constructor with alternate intake angle
        Preset(String name, double elevatorHeight, double endEffectorAngle, double intakeAngle, double alternateIntakeAngle) {
            this(
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
                new LoggedTunableNumber("Superstructure/" + name + "/Pivot", endEffectorAngle),
                createToggleableIntakeSupplier(name, intakeAngle, alternateIntakeAngle));
        }
      
        // Constructor without alternate intake angle (default behavior)
        Preset(String name, double elevatorHeight, double endEffectorAngle, double intakeAngle) {
            this(
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
                new LoggedTunableNumber("Superstructure/" + name + "/Pivot", endEffectorAngle),
                new LoggedTunableNumber("Superstructure/" + name + "/Intake", intakeAngle));
        }
        
        /**
         * Creates a supplier that toggles between default and alternate intake angles.
         */
        private static DoubleSupplier createToggleableIntakeSupplier(String name, double defaultAngle, double alternateAngle) {
            LoggedTunableNumber defaultTunable = new LoggedTunableNumber("Superstructure/" + name + "/Intake", defaultAngle);
            LoggedTunableNumber alternateTunable = new LoggedTunableNumber("Superstructure/" + name + "/IntakeAlt", alternateAngle);
            
            return () -> intakeToggleState ? alternateTunable.get() : defaultTunable.get();
        }
    }
} 