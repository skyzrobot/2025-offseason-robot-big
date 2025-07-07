package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ntext.NTParameter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

//TODO: change motion logic and reset command afterwards

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  @Setter
  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  public boolean hasDeployed() {
    return systemState == SystemState.DEPLOYING;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SystemState newState = handleStateTransition();

    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/SystemState", newState.toString());

    if (newState != systemState) {
      systemState = newState;
    }

    switch (systemState) {
      case IDLING:
        io.setTargetPosition(ClimberParamsNT.IdleAngle.getValue());
        io.setRollerVoltage(Volts.of(0.0));
        break;
      case DEPLOYING:
        io.setTargetPosition(ClimberParamsNT.DeployAngle.getValue());
        io.setRollerVoltage(Volts.of(ClimberParamsNT.RollerLockOnVoltage.getValue()));
        break;
      case CLIMBING:
        io.setTargetPosition(ClimberParamsNT.ClimbAngle.getValue());
        io.setRollerVoltage(Volts.of(ClimberParamsNT.RollerHoldVoltage.getValue()));
        break;
    }

    if (ClimberParamsNT.isAnyChanged())
      io.setParams(
          ClimberParamsNT.Kp.getValue(),
          ClimberParamsNT.Ki.getValue(),
          ClimberParamsNT.Kd.getValue(),
          ClimberParamsNT.CruiseVelocity.getValue(),
          ClimberParamsNT.Acceleration.getValue(),
          ClimberParamsNT.Jerk.getValue()
      );
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case DEPLOY -> SystemState.DEPLOYING;
      case CLIMB -> SystemState.CLIMBING;
      case IDLE -> SystemState.IDLING;
    };
  }

  public void resetPosition() {
    io.resetPosition();
  }

  public void setCoast() {
    io.setCoast();
  }

  public void setBrake() {
    io.setBrake();
  }

  public enum WantedState {
    DEPLOY,
    CLIMB,
    IDLE
  }

  public enum SystemState {
    DEPLOYING,
    CLIMBING,
    IDLING
  }


  @NTParameter(tableName = "Params/Subsystems/Climber")
  public static final class ClimberParams {
    static final double RollerLockOnVoltage = -12.0;
    static final double RollerHoldVoltage = -1.0;

    static final double Kp = 18.0;
    static final double Ki = 0.0;
    static final double Kd = 0.01;

    static final double CruiseVelocity = 10000.0;
    static final double Acceleration = 10000.0;
    static final double Jerk = 0.0;

    static final double DeployAngle = 375.0;
    static final double IdleAngle = 375.0;
    static final double ClimbAngle = -175.0;
  }

}
