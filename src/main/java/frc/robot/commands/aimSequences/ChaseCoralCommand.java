package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.Logging;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseCoralCommand extends Command {
  private final Swerve swerve;
  //  private final PhotonVisionSubsystem vision;
  private final PhotonCamera camera = new PhotonCamera("pv-cam1");

  private final PIDController driveController;
  private final PIDController turnController;
  private final TimeDelayedBoolean isBlind = new TimeDelayedBoolean(0.5);
  State state = State.ACTIVE_CHASING;
  private Pair<Double, Double> chaseTarget;

  public ChaseCoralCommand(Swerve swerve, PhotonVisionSubsystem vision) {
    this.swerve = swerve;
//    this.vision = vision;

    driveController = new PIDController(
        ChaseCoralCommandParamsNT.driveKp.getValue(),
        ChaseCoralCommandParamsNT.driveKi.getValue(),
        ChaseCoralCommandParamsNT.driveKd.getValue()
    );
    turnController = new PIDController(
        ChaseCoralCommandParamsNT.turnKp.getValue(),
        ChaseCoralCommandParamsNT.turnKi.getValue(),
        ChaseCoralCommandParamsNT.turnKd.getValue()
    );

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    driveController.setP(ChaseCoralCommandParamsNT.driveKp.getValue());
    driveController.setI(ChaseCoralCommandParamsNT.driveKi.getValue());
    driveController.setD(ChaseCoralCommandParamsNT.driveKd.getValue());

    turnController.setP(ChaseCoralCommandParamsNT.turnKp.getValue());
    turnController.setI(ChaseCoralCommandParamsNT.turnKi.getValue());
    turnController.setD(ChaseCoralCommandParamsNT.turnKd.getValue());

    driveController.reset();
    turnController.reset();
    chaseTarget = Pair.of(0.0, 0.0);
    state = State.ACTIVE_CHASING;
  }

  @Override
  public void execute() {
    // handle state transition
//    if (!vision.getAllRawDetections().isEmpty())
    PhotonTrackedTarget target;
    if (camera.getLatestResult().hasTargets() && camera.getLatestResult().getBestTarget().getPitch() > -10.0) state = State.ACTIVE_CHASING;
    else state = State.BLIND_CHASING;

    // get
    Rotation2d currentAngle = swerve.getEstimatedPose().getRotation().toRotation2d();

    // run state
    switch (state) {
      case ACTIVE_CHASING -> {
        Logging.info("Commands/ChaseCoralCommand", "Active Chasing!");
//        PhotonVisionSubsystem.RawDetection detection = vision.getLargestTarget();
        if (!camera.getLatestResult().hasTargets()) {
          state = State.BLIND_CHASING;
          return;
        }

        var detection = camera.getLatestResult().getBestTarget();
        double forwardVel = -driveController.calculate(detection.getPitch(), ChaseCoralCommandParamsNT.chasePitchSetpoint.getValue());
        forwardVel = MathUtil.clamp(forwardVel, 0.0, ChaseCoralCommandParamsNT.activeChaseMaxVelocityMps.getValue());

        double angVel = turnController.calculate(detection.getYaw(), ChaseCoralCommandParamsNT.chaseYawSetpoint.getValue());

        chaseTarget = Pair.of(forwardVel, angVel);
      }
      case BLIND_CHASING -> {
        Logging.info("Commands/ChaseCoralCommand", "Blind Chasing!");
        chaseTarget = Pair.of(MathUtil.clamp(chaseTarget.getFirst(), 0.0, ChaseCoralCommandParamsNT.blindChaseMaxVelocityMps.getValue()), 0.0);
      }
    }

    // run target
    swerve.runTwist(new ChassisSpeeds(chaseTarget.getFirst(), 0.0, chaseTarget.getSecond()));

    // logging
    Logger.recordOutput("Commands/ChaseCoralCommand/Linvel", chaseTarget.getFirst());
    Logger.recordOutput("Commands/ChaseCoralCommand/AngCurrent", currentAngle.getRadians());

  }

  @Override
  public void end(boolean interrupted) {
    swerve.runStop();
  }

  @Override
  public boolean isFinished() {
    return isBlind.update(state == State.BLIND_CHASING, ChaseCoralCommandParamsNT.blindChaseMaxTimeSeconds.getValue());
  }

  private enum State {
    ACTIVE_CHASING, BLIND_CHASING
  }

  @NTParameter(tableName = "Params/Commands/ChaseCoralCommand")
  public static class ChaseCoralCommandParams {
    static final double driveKp = 0.5;
    static final double driveKi = 0.0;
    static final double driveKd = 0.0;

    static final double turnKp = 0.5;
    static final double turnKi = 0.0;
    static final double turnKd = 0.0;

    static final double chasePitchSetpoint = -14.0;
    static final double chaseYawSetpoint = 0.0;

    static final double activeChaseMaxVelocityMps = 2.0;
    static final double blindChaseMaxTimeSeconds = 0.5;
    static final double blindChaseMaxVelocityMps = 1.5;
  }
}
