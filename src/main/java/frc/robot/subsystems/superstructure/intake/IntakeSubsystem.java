package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.intake.IntakePivotIOInputsAutoLogged;
import frc.robot.utils.LoggedTracer;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;

public class IntakeSubsystem {
    private final IntakePivotIO intakePivotIO;
    private final RollerIO intakeRollerIO;
    private final RollerIO indexRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged indexRollerIOInputs = new RollerIOInputsAutoLogged();
    private final BeambreakIO BBIO;
    private final BeambreakIOInputsAutoLogged BBInputs = new BeambreakIOInputsAutoLogged();


    @Getter
    @AutoLogOutput(key = "Intake/setPoint")
    private double wantedAngle = 0.0;
    @Getter
    @AutoLogOutput(key = "Intake/atGoal")
    private boolean atGoal = false;

    @Getter
    @Setter
    @AutoLogOutput(key = "Intake/indexRollerHasCoral")
    private boolean indexRollerHasCoral = false;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            RollerIO intakeRollerIO,
            RollerIO indexRollerIO,
            BeambreakIO BBIO
    ) {
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
        this.indexRollerIO = indexRollerIO;
        this.BBIO = BBIO;
    }


    public void periodic() {

        BBIO.updateInputs(BBInputs);
        intakePivotIO.updateInputs(intakePivotIOInputs);
        intakeRollerIO.updateInputs(intakeRollerIOInputs);
        indexRollerIO.updateInputs(indexRollerIOInputs);

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);
        Logger.processInputs("Intake/Roller", intakeRollerIOInputs);
        Logger.processInputs("Intake/IndexRoller", indexRollerIOInputs);
        Logger.processInputs("Intake/Roller/Beambreak", BBInputs);
        atGoal = isNearAngle(wantedAngle, IntakeConstants.INTAKE_PIVOT_TOLERANCE.get());
        intakePivotIO.setPivotAngle(wantedAngle);

        //TODO: add Debouncer or filter to prevent false positives
        if (RobotBase.isReal()) {
            indexRollerHasCoral = BBInputs.isBeambreakOn;
            SmartDashboard.putBoolean("GamePiece/IndexRollerHasCoral", indexRollerHasCoral);
        }
        LoggedTracer.record("Intake");
    }

    /**
     * Checks if the arm is near a target angle
     *
     * @param targetAngleDeg The target angle in degrees
     * @return True if the arm is within tolerance of the target
     */
    public boolean isNearAngle(double targetAngleDeg, double tolerance) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, tolerance);
    }

    /**
     * Checks if the intake is in a dangerous position
     *
     * @return True if the intake is in a dangerous position
     */
    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentAngleDeg < RobotConstants.IntakeConstants.INTAKE_DANGER_ZONE - 2;
    }

    // Basic control methods
    public void setPivotAngle(DoubleSupplier angleDeg) {
        wantedAngle = angleDeg.getAsDouble();
    }

    public void setPivotVoltage(double voltage) {
        intakePivotIO.setMotorVoltage(voltage);
    }

    public void setIntakeRollerVoltage(DoubleSupplier voltage) {
        intakeRollerIO.setVoltage(voltage.getAsDouble());
    }

    public void stopRoller() {
        intakeRollerIO.stop();
    }

    public double getCurrentAngle() {
        return intakePivotIOInputs.currentAngleDeg;
    }

    public void setIndexRollerVoltage(DoubleSupplier voltage) {
        indexRollerIO.setVoltage(voltage.getAsDouble());
    }

    public void stopIndexRoller() {
        indexRollerIO.stop();
    }

}