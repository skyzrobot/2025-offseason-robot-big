package frc.robot.subsystems.superstructure.endeffectorarm;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.EndEffectorArmParamsNT;

public interface EndEffectorArmPivotIO {
    default void updateInputs(EndEffectorArmPivotIOInputs inputs) {
    }

    default void setPivotAngle(double targetAngleDeg) {
    }

    default void resetAngle(double resetAngleDeg) {
    }

    default void updateGains(double kP, double kI, double kD, double kA, double kV, double kS, double kG) {
    }

    @AutoLog
    class EndEffectorArmPivotIOInputs {
        public double targetAngleDeg = 0.0;
        public double currentAngleDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double endEffectorArmPivotKP = EndEffectorArmParamsNT.pivotKP.getValue();
        public double endEffectorArmPivotKI = EndEffectorArmParamsNT.pivotKI.getValue();
        public double endEffectorArmPivotKD = EndEffectorArmParamsNT.pivotKD.getValue();
        public double endEffectorArmPivotKA = EndEffectorArmParamsNT.pivotKA.getValue();
        public double endEffectorArmPivotKV = EndEffectorArmParamsNT.pivotKV.getValue();
        public double endEffectorArmPivotKS = EndEffectorArmParamsNT.pivotKS.getValue();
        public double endEffectorArmPivotKG = EndEffectorArmParamsNT.pivotKG.getValue();
    }
} 