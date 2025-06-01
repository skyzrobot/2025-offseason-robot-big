package frc.robot.subsystems.superstructure.endeffectorarm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotConstants.EndEffectorArmConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmPivotIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;    
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.TimeDelayedBoolean;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.RobotConstants.EndEffectorArmConstants.*;
import static frc.robot.RobotConstants.EndEffectorArmConstants.EndEffectorArmRollerGainsClass.*;

import java.util.function.DoubleSupplier;

public class EndEffectorArmSubsystem extends SubsystemBase {
    public static final String NAME = "EndEffectorArm";
    // IO devices and their inputs
    private final EndEffectorArmPivotIO armPivotIO;
    private final EndEffectorArmPivotIOInputsAutoLogged	 armPivotIOInputs = new EndEffectorArmPivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();

    // Beambreak sensors for coral and algae detection
    private final BeambreakIO coralBeambreakIO;
    private final BeambreakIO algaeBeambreakIO;
    private final BeambreakIOInputsAutoLogged coralBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged algaeBeambreakInputs = new BeambreakIOInputsAutoLogged();

    // Roller motor control
    private final RollerIO rollerIO;

    @Getter@AutoLogOutput(key = "EndEffectorArm/setPoint")
    private double wantedAngle = 0.0;
    @Getter@AutoLogOutput(key = "EndEffectorArm/atGoal")
    private boolean atGoal = false;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasCoral")
    private boolean hasCoral = false;
    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasAlgae")
    private boolean hasAlgae = false;

    public EndEffectorArmSubsystem(
            EndEffectorArmPivotIO armPivotIO,
            RollerIO rollerIO,
            BeambreakIO coralBeambreakIO,
            BeambreakIO algaeBeambreakIO) {
        this.armPivotIO = armPivotIO;
        this.rollerIO = rollerIO;
        this.coralBeambreakIO = coralBeambreakIO;
        this.algaeBeambreakIO = algaeBeambreakIO;

        // Apply initial PID gains
        rollerIO.updateConfigs(
            END_EFFECTOR_ARM_ROLLER_KP.get(),
            END_EFFECTOR_ARM_ROLLER_KI.get(),
            END_EFFECTOR_ARM_ROLLER_KD.get(),
            END_EFFECTOR_ARM_ROLLER_KA.get(),
            END_EFFECTOR_ARM_ROLLER_KV.get(),
            END_EFFECTOR_ARM_ROLLER_KS.get()
        );
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        armPivotIO.updateInputs(armPivotIOInputs);
        coralBeambreakIO.updateInputs(coralBeambreakInputs);
        algaeBeambreakIO.updateInputs(algaeBeambreakInputs);
        rollerIO.updateInputs(armRollerIOInputs);

        if (RobotBase.isReal()) {
            // Update gamepiece tracking
            //TODO: add Debouncer or filter to prevent false positives
            hasCoral = coralBeambreakInputs.isBeambreakOn;
            hasAlgae = algaeBeambreakInputs.isBeambreakOn;
            SmartDashboard.putBoolean("GamePiece/EEHasCoral", hasCoral);
            SmartDashboard.putBoolean("GamePiece/EEHasAlgae", hasAlgae);
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Pivot", armPivotIOInputs);
        Logger.processInputs(NAME + "/Coral Beambreak", coralBeambreakInputs);
        Logger.processInputs(NAME + "/Algae Beambreak", algaeBeambreakInputs);
        Logger.processInputs(NAME + "/Roller", armRollerIOInputs);

        // Update goal status and set pivot angle
        atGoal = isNearAngle(wantedAngle,EndEffectorArmConstants.END_EFFECTOR_ARM_PIVIOT_TOLERANCE.get());
        armPivotIO.setPivotAngle(wantedAngle);

        // Update tunable numbers if tuning is enabled
        if (RobotConstants.TUNING) {
            // Update roller PID gains if tuning is enabled
            rollerIO.updateConfigs(
                END_EFFECTOR_ARM_ROLLER_KP.get(),
                END_EFFECTOR_ARM_ROLLER_KI.get(),
                END_EFFECTOR_ARM_ROLLER_KD.get(),
                END_EFFECTOR_ARM_ROLLER_KA.get(),
                END_EFFECTOR_ARM_ROLLER_KV.get(),
                END_EFFECTOR_ARM_ROLLER_KS.get()
            );
        }
        LoggedTracer.record("EndEffectorArm");
    }


    public boolean isNearAngle(double targetAngleDeg, double tolerance) {
        return MathUtil.isNear(targetAngleDeg, armPivotIOInputs.currentAngleDeg, tolerance);
    }

    // Basic control methods
    public void setPivotAngle(DoubleSupplier angleDeg) {
        wantedAngle = angleDeg.getAsDouble();
    }

    public void setRollerVoltage(DoubleSupplier voltage) {
        rollerIO.setVoltage(voltage.getAsDouble());
    }

    public void stopRoller() {
        rollerIO.stop();
    }

    public double getCurrentAngle() {
        return armPivotIOInputs.currentAngleDeg;
    }
} 