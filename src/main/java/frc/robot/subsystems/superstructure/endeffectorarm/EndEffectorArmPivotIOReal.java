package frc.robot.subsystems.superstructure.endeffectorarm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.EndEffectorArmParamsNT;
import frc.robot.RobotConstants;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmPivotIOReal implements EndEffectorArmPivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.EndEffectorArmConstants.END_EFFECTOR_ARM_PIVOT_MOTOR_ID,
            RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final CANcoder CANcoder = new CANcoder(END_EFFECTOR_ARM_ENCODER_ID, RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    double targetAngleDeg = 135.0;

    public EndEffectorArmPivotIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //initialize CANcoder
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = EndEffectorArmParamsNT.encoderOffset.getValue();
        CANconfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoder.getConfigurator().apply(CANconfig);
        //integrate with fused encoder
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = END_EFFECTOR_ARM_ENCODER_ID;
        config.Feedback.RotorToSensorRatio = ROTOR_SENSOR_RATIO;

        // Configure Motion Magic using NTParam values
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = EndEffectorArmParamsNT.motionMagicCruiseVelocity.getValue() ; // Convert degrees/sec to rotations/sec
        motionMagicConfigs.MotionMagicAcceleration = EndEffectorArmParamsNT.motionMagicAcceleration.getValue(); // Convert degrees/sec^2 to rotations/sec^2
        motionMagicConfigs.MotionMagicJerk = EndEffectorArmParamsNT.motionMagicJerk.getValue(); // Convert degrees/sec^3 to rotations/sec^3
        config.withMotionMagic(motionMagicConfigs);

        config.withSlot0(new Slot0Configs()
                .withKP(EndEffectorArmParamsNT.pivotKP.getValue())
                .withKI(EndEffectorArmParamsNT.pivotKI.getValue())
                .withKD(EndEffectorArmParamsNT.pivotKD.getValue())
                .withKA(EndEffectorArmParamsNT.pivotKA.getValue())
                .withKV(EndEffectorArmParamsNT.pivotKV.getValue())
                .withKS(EndEffectorArmParamsNT.pivotKS.getValue())
                .withKG(EndEffectorArmParamsNT.pivotKG.getValue())
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withGravityType(GravityTypeValue.Arm_Cosine)
        );

        motor.getConfigurator().apply(config);

        motor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                motorVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EndEffectorArmPivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                motorVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.motorVolts = motorVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.currentAngleDeg = talonPosToAngle(currentPositionRot.getValueAsDouble()) - 63;
        inputs.targetAngleDeg = targetAngleDeg;

        if (RobotConstants.TUNING) {
            // Check if NTParams have changed and update gains accordingly
            if (EndEffectorArmParamsNT.isAnyChanged()) {
                updateGains(
                    EndEffectorArmParamsNT.pivotKP.getValue(),
                    EndEffectorArmParamsNT.pivotKI.getValue(), 
                    EndEffectorArmParamsNT.pivotKD.getValue(),
                    EndEffectorArmParamsNT.pivotKA.getValue(),
                    EndEffectorArmParamsNT.pivotKV.getValue(),
                    EndEffectorArmParamsNT.pivotKS.getValue(),
                    EndEffectorArmParamsNT.pivotKG.getValue()
                );
                
                // Update Motion Magic parameters if they changed
                var motionMagicConfigs = new MotionMagicConfigs();
                motionMagicConfigs.MotionMagicCruiseVelocity = EndEffectorArmParamsNT.motionMagicCruiseVelocity.getValue() / 360.0;
                motionMagicConfigs.MotionMagicAcceleration = EndEffectorArmParamsNT.motionMagicAcceleration.getValue() / 360.0;
                motionMagicConfigs.MotionMagicJerk = EndEffectorArmParamsNT.motionMagicJerk.getValue() / 360.0;
                motor.getConfigurator().apply(motionMagicConfigs);
            }
            
            // Log current values for tuning
            inputs.endEffectorArmPivotKP = EndEffectorArmParamsNT.pivotKP.getValue();
            inputs.endEffectorArmPivotKI = EndEffectorArmParamsNT.pivotKI.getValue();
            inputs.endEffectorArmPivotKD = EndEffectorArmParamsNT.pivotKD.getValue();
            inputs.endEffectorArmPivotKA = EndEffectorArmParamsNT.pivotKA.getValue();
            inputs.endEffectorArmPivotKV = EndEffectorArmParamsNT.pivotKV.getValue();
            inputs.endEffectorArmPivotKS = EndEffectorArmParamsNT.pivotKS.getValue();
            inputs.endEffectorArmPivotKG = EndEffectorArmParamsNT.pivotKG.getValue();
        }
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kA, double kV, double kS, double kG) {
        motor.getConfigurator().apply(new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKA(kA)
                .withKV(kV)
                .withKS(kS)
                .withKG(kG)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withGravityType(GravityTypeValue.Arm_Cosine));
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.targetAngleDeg = targetAngleDeg;
        motor.setControl(motionMagicRequest.withPosition(angleToTalonPos(targetAngleDeg + 63)));
    }

    private double angleToTalonPos(double angleDeg) {
        return (angleDeg / 360) * 1;
    }

    private double talonPosToAngle(double rotations) {
        return rotations * 360;
    }
}
