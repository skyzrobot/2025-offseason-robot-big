package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  default void updateInputs(ClimberIOInputs inputs) {
  }

  default void setTargetPosition(double targetPositionDeg) {
  }

  default void setRollerVoltage(Voltage voltage) {
  }

  default void resetPosition() {
  }

  default void setCoast() {
  }

  default void setBrake() {
  }

  default void setParams(double kP, double kI, double kD, double cruiseVelocity, double acceleration, double jerk) {

  }

  @AutoLog
  class ClimberIOInputs {
    public double currentPositionDeg = 0.0;
    public double targetPositionDeg = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
  }
}
