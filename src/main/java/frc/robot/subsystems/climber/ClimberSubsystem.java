package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
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

  // Auto climb detection variables
  private boolean currentSpikeDetected = false;
  private double spikeDetectionStartTime = -1;
  
  @Setter
  private boolean autoClimbEnabled = true;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  public boolean hasDeployed() {
    return systemState == SystemState.DEPLOYING;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    // Auto climb detection logic
    if (autoClimbEnabled && systemState == SystemState.DEPLOYING) {
      detectAutoClimb();
    }
    
    SystemState newState = handleStateTransition();

    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/SystemState", newState.toString());
    Logger.recordOutput("Climber/AutoClimb/CurrentSpikeDetected", currentSpikeDetected);
    Logger.recordOutput("Climber/AutoClimb/SpikeDetectionTime", 
        spikeDetectionStartTime > 0 ? Timer.getFPGATimestamp() - spikeDetectionStartTime : 0.0);
    Logger.recordOutput("Climber/AutoClimb/AutoClimbEnabled", autoClimbEnabled);

    if (newState != systemState) {
      systemState = newState;
      // Reset auto climb detection when transitioning to new state
      resetAutoClimbDetection();
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

  private void detectAutoClimb() {
    double currentTime = Timer.getFPGATimestamp();
    
    System.out.println("Climber Auto Climb Detection - Current: " + inputs.rollerStatorCurrentAmps + 
                      " Spike Detected: " + currentSpikeDetected + 
                      " Detection Time: " + (spikeDetectionStartTime > 0 ? currentTime - spikeDetectionStartTime : 0));
    
    // Check if current is above spike threshold
    if (inputs.rollerStatorCurrentAmps >= ClimberParamsNT.SpikeCurrentThreshold.getValue()) {
      // Start timing the spike if not already started
      if (!currentSpikeDetected) {
        currentSpikeDetected = true;
        spikeDetectionStartTime = currentTime;
        System.out.println("Climber: Current spike detected! Current: " + inputs.rollerStatorCurrentAmps);
      } else {
        // Check if spike has been sustained for debounce duration
        double spikeDuration = currentTime - spikeDetectionStartTime;
        if (spikeDuration >= ClimberParamsNT.SpikeDebouceDuration.getValue()) {
          // Auto transition to climbing
          wantedState = WantedState.CLIMB;
          System.out.println("Climber: Auto climb triggered! Current spike sustained for " + spikeDuration + " seconds");
        }
      }
    } else {
      // Current dropped below threshold - reset detection
      if (currentSpikeDetected) {
        System.out.println("Climber: Current spike ended, resetting detection");
        resetAutoClimbDetection();
      }
    }
  }
  
  private void resetAutoClimbDetection() {
    currentSpikeDetected = false;
    spikeDetectionStartTime = -1;
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
  
  public void disableAutoClimb() {
    autoClimbEnabled = false;
    resetAutoClimbDetection();
  }
  
  public void enableAutoClimb() {
    autoClimbEnabled = true;
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
    static final double RollerLockOnVoltage = 12.0;
    static final double RollerHoldVoltage = 1.0;

    static final double Kp = 18.0;
    static final double Ki = 0.0;
    static final double Kd = 0.01;

    static final double CruiseVelocity = 10000.0;
    static final double Acceleration = 10000.0;
    static final double Jerk = 0.0;

    static final double DeployAngle = 0;
    static final double IdleAngle = 0;
    static final double ClimbAngle = -650.0;
    
    // Auto climb parameters
    static final double SpikeCurrentThreshold = 20.0;  // Amps threshold to detect current spike
    static final double SpikeDebouceDuration = 0.2;  // Seconds to debounce the spike before auto climb
  }

}
