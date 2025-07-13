package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lib.ntext.NTParameter;

public class ChaseCoralCommand extends Command {
  private final Swerve swerve;
  private final PIDController driveController;
  private final PIDController turnController;
  private final TimeDelayedBoolean blindTimer = new TimeDelayedBoolean(0.5);
  private State state = State.ACTIVE_CHASING;
  private Pose2d robotPose = new Pose2d();
  private Rotation2d lastDirection = Rotation2d.fromDegrees(0.0);
  private double forwardVel = 0.0;
  private double turnVel = 0.0;
  private Integer targetCoralId = null;
  public ChaseCoralCommand(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

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
    turnController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize() {
    driveController.reset();
    turnController.reset();
    targetCoralId = null;
    state = State.ACTIVE_CHASING;
    lastDirection = RobotStateRecorder
        .getPoseDriverRobotCurrent().toPose2d()
        .getRotation();
  }

  @Override
  public void execute() {
    robotPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();

    // pick a new target if we don’t have one
    if (targetCoralId == null) {
      RobotStateRecorder
          .getNearestCoralInSight()
          .ifPresent(info -> targetCoralId = info.id);
    }

    Translation2d targetTranslation = null;
    if (targetCoralId != null) {
      var opt = RobotStateRecorder.getCoralById(targetCoralId);
      if (opt.isPresent()) {
        targetTranslation = opt.get().translation;
        state = State.ACTIVE_CHASING;
      } else {
        state = State.BLIND_CHASING;
      }
    } else {
      state = State.BLIND_CHASING;
    }

    if (state == State.ACTIVE_CHASING) {
      // compute distance and bearing to coral
      Translation2d toTarget = targetTranslation.minus(robotPose.getTranslation());
      double distance = toTarget.getNorm();
      lastDirection = toTarget.getAngle();

      // drive toward it
      double rawDrive = driveController.calculate(distance, 0.0);
      forwardVel = MathUtil.clamp(
          -rawDrive,
          0.0,
          ChaseCoralCommandParamsNT.activeChaseMaxVelocityMps.getValue()
      );

      // turn to face it
      double robotAngle = robotPose.getRotation().getRadians();
      turnVel = turnController.calculate(
          robotAngle,
          lastDirection.getRadians()
      );

      blindTimer.reset();
    } else {
      // keep moving in last known direction
      forwardVel = MathUtil.clamp(
          forwardVel,
          0.0,
          ChaseCoralCommandParamsNT.blindChaseMaxVelocityMps.getValue()
      );
      turnVel = 0.0;
    }

    // build field‐relative velocity vector
    double vx = forwardVel * lastDirection.getCos();
    double vy = forwardVel * lastDirection.getSin();
    swerve.runTwist(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            turnVel,
            robotPose.getRotation()
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.runStop();
  }

  @Override
  public boolean isFinished() {
    // finish if we've been blind too long
    return blindTimer.update(
        state == State.BLIND_CHASING,
        ChaseCoralCommandParamsNT.blindChaseMaxTimeSeconds.getValue()
    );
  }

  private enum State {ACTIVE_CHASING, BLIND_CHASING}

  @NTParameter(tableName = "Params/Commands/ChaseCoralCommand")
  public static class ChaseCoralCommandParams {
    static final double driveKp = 2.5;
    static final double driveKi = 0.0;
    static final double driveKd = 0.1;
    static final double turnKp = 6.0;
    static final double turnKi = 0.0;
    static final double turnKd = 0.2;
    static final double activeChaseMaxVelocityMps = 2.5;
    static final double blindChaseMaxTimeSeconds = 0.3;
    static final double blindChaseMaxVelocityMps = 1.5;
  }
}
