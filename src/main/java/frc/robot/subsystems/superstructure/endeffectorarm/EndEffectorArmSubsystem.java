package frc.robot.subsystems.superstructure.endeffectorarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EndEffectorArmParamsNT;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;    
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.utils.LoggedTracer;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

import java.util.function.DoubleSupplier;

public class EndEffectorArmSubsystem {
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

    @Getter
    @AutoLogOutput(key = "EndEffectorArm/setPoint")
    private double wantedAngle = 135.0;
    @Getter@AutoLogOutput(key = "EndEffectorArm/atGoal")
    private boolean atGoal = false;
    @AutoLogOutput(key = "EndEffectorArm/stopDueToLimit")
    private boolean stopDueToLimit = false;

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

        // Apply initial PID gains using NTParam values
        rollerIO.updateConfigs(
            EndEffectorArmParamsNT.rollerKP.getValue(),
            EndEffectorArmParamsNT.rollerKI.getValue(),
            EndEffectorArmParamsNT.rollerKD.getValue(),
            EndEffectorArmParamsNT.rollerKA.getValue(),
            EndEffectorArmParamsNT.rollerKV.getValue(),
            EndEffectorArmParamsNT.rollerKS.getValue()
        );
    }

    public void periodic() {
        // Update inputs from hardware
        armPivotIO.updateInputs(armPivotIOInputs);
        coralBeambreakIO.updateInputs(coralBeambreakInputs);
        algaeBeambreakIO.updateInputs(algaeBeambreakInputs);
        rollerIO.updateInputs(armRollerIOInputs);

        // Check if angle exceeds maximum limit
        if (wantedAngle > EndEffectorArmParamsNT.maxAngleDegrees.getValue()) {
            stopDueToLimit = true;
            System.out.println("EndEffectorArm setpoint " + wantedAngle + " exceeds maximum angle of " + 
                EndEffectorArmParamsNT.maxAngleDegrees.getValue() + " degrees");
            throw new IllegalArgumentException("EndEffectorArm setpoint " + wantedAngle + " exceeds maximum angle of " + 
                EndEffectorArmParamsNT.maxAngleDegrees.getValue() + " degrees");
        } else if (stopDueToLimit) {
            // Reset stopDueToLimit if angle is now valid
            stopDueToLimit = false;
        }

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
        atGoal = isNearAngle(wantedAngle, EndEffectorArmParamsNT.pivotTolerance.getValue());
        if (!stopDueToLimit) {
            armPivotIO.setPivotAngle(wantedAngle);
        }

        // Update tunable numbers if tuning is enabled
        if (RobotConstants.TUNING && EndEffectorArmParamsNT.isAnyChanged()) {
            // Update roller PID gains if tuning is enabled
            rollerIO.updateConfigs(
                EndEffectorArmParamsNT.rollerKP.getValue(),
                EndEffectorArmParamsNT.rollerKI.getValue(),
                EndEffectorArmParamsNT.rollerKD.getValue(),
                EndEffectorArmParamsNT.rollerKA.getValue(),
                EndEffectorArmParamsNT.rollerKV.getValue(),
                EndEffectorArmParamsNT.rollerKS.getValue()
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