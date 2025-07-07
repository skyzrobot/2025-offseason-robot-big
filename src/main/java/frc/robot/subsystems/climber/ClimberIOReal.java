package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;
import lib.ironpulse.utils.Logging;

import static edu.wpi.first.units.Units.Volts;


public class ClimberIOReal implements ClimberIO {
  private final TalonFX motor = new TalonFX(
      RobotConstants.ClimberConstants.CLIMBER_MOTOR_ID,
      RobotConstants.CLIMBER_CAN_BUS
  );
  private final TalonFX roller = new TalonFX(
      RobotConstants.ClimberConstants.CLIMBER_ROLLER_MOTOR_ID,
      RobotConstants.CLIMBER_CAN_BUS
  );

  private final StatusSignal<Angle> currentPositionRot = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Current> rollerStatorCurrentAmps = roller.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final double CLIMBER_RATIO = RobotConstants.ClimberConstants.CLIMBER_RATIO;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private MotionMagicConfigs motionMagicConfigs;
  private double targetPositionDeg = 0.0;

  public ClimberIOReal() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocityRotPerSec,
        tempCelsius,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        rollerStatorCurrentAmps,
        currentPositionRot
    );

    motor.getConfigurator().apply(config);
    motor.setPosition(0.0);
    motor.optimizeBusUtilization();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    roller.getConfigurator().apply(rollerConfig);
    roller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        currentPositionRot,
        velocityRotPerSec,
        tempCelsius,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        rollerStatorCurrentAmps
    );

    inputs.currentPositionDeg = currentPositionRot.getValueAsDouble() * 360 / CLIMBER_RATIO;
    inputs.velocityRotationsPerSec = velocityRotPerSec.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.targetPositionDeg = targetPositionDeg;
    inputs.rollerStatorCurrentAmps = rollerStatorCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setTargetPosition(double targetPositionDeg) {
    motor.setControl(positionRequest.withPosition(targetPositionDeg * CLIMBER_RATIO / 360));
    this.targetPositionDeg = targetPositionDeg;
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    roller.setVoltage(voltage.in(Volts));
  }

  @Override
  public void resetPosition() {
    motor.setPosition(0.0);
  }

  @Override
  public void setCoast() {
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setBrake() {
    System.out.println(motor.setNeutralMode(NeutralModeValue.Brake));
  }

  public void setParams(double kP, double kI, double kD, double cruiseVelocity, double acceleration, double jerk) {
    config.withSlot0(new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD));
    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicJerk = jerk;
    config.withMotionMagic(motionMagicConfigs);
    motor.getConfigurator().apply(config);

    Logging.info(
        "Climber", "Setting Params: %.2f %.2f %.2f %.2f %.2f %.2f",
        kP, kI, kD, cruiseVelocity, acceleration, jerk
    );
  }
}
