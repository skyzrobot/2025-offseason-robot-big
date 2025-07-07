package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class ReefAimCommand extends Command {
  private final static String kTag = "Commands/ReefAimCommand";
  private final Swerve swerve;
  private final IndicatorSubsystem indicatorSubsystem;
  private boolean rightReef; // true if shooting right reef
  private boolean translationOnTarget = false;
  private boolean rotationOnTarget = false;
  private boolean translationStationary = false;
  private boolean rotationStationary = false;
  private Pose2d poseWorldRobot, velocityRobot, tagPose, poseWorldTarget, finalDestinationPose;
  private PIDController translationController;
  private PIDController rotationController;


  public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
    this.indicatorSubsystem = indicatorSubsystem;
    this.swerve = swerve;

    translationController = new PIDController(
        ReefAimCommandParamsNT.translationKp.getValue(),
        ReefAimCommandParamsNT.translationKi.getValue(),
        ReefAimCommandParamsNT.translationKd.getValue()
    );
    rotationController = new PIDController(
        ReefAimCommandParamsNT.rotationKp.getValue(),
        ReefAimCommandParamsNT.rotationKi.getValue(),
        ReefAimCommandParamsNT.rotationKd.getValue()
    );
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // tuning
    if (RobotConstants.TUNING) {
      translationController.setP(ReefAimCommandParamsNT.translationKp.getValue());
      translationController.setI(ReefAimCommandParamsNT.translationKi.getValue());
      translationController.setIZone(ReefAimCommandParamsNT.translationKiZone.getValue());
      translationController.setD(ReefAimCommandParamsNT.translationKd.getValue());
      translationController.setTolerance(
          ReefAimCommandParamsNT.translationOnTargetToleranceMeter.getValue(),
          ReefAimCommandParamsNT.translationOnTargetVelocityMetersPerSecond.getValue()
      );

      rotationController.setP(ReefAimCommandParamsNT.rotationKp.getValue());
      rotationController.setI(ReefAimCommandParamsNT.rotationKi.getValue());
      rotationController.setIZone(ReefAimCommandParamsNT.rotationKiZone.getValue());
      rotationController.setD(ReefAimCommandParamsNT.rotationKd.getValue());
      rotationController.setTolerance(
          ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue() / 180.0f * Math.PI,
          ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue() / 180.0f * Math.PI
      );
    }

    // get current state
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    velocityRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

    // calculate destination
    tagPose = AimGoalSupplier.getNearestTag(poseWorldRobot);

    // choose target based on game piece
    if (DestinationSupplier.getInstance().getCurrentGamePiece() == DestinationSupplier.GamePiece.ALGAE_INTAKING) {
      finalDestinationPose = AimGoalSupplier.getFinalAlgaeTarget(tagPose);
    } else {
      rightReef = DestinationSupplier.getInstance().getCurrentBranch();
      finalDestinationPose = AimGoalSupplier.getFinalCoralTarget(tagPose, rightReef);
    }

    // Now that finalDestinationPose is set, we can get the drive target
    poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);

    // PID init with field-relative velocities
    rotationController.enableContinuousInput(0, Math.PI * 2);
    translationController.reset();
    rotationController.reset();
    indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
  }

  @Override
  public void execute() {
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    velocityRobot = RobotStateRecorder.getVelocityRobotCurrent();

    // compute translation error, tu
    Translation2d pRT = poseRobotTarget.getTranslation();
    double pRT_norm = pRT.getNorm();
    Rotation2d pRT_dir = pRT.getAngle();
    // NOTE: as pRT_norm is always positive, then vRT_norm is always negative.
    // to make the robot move along but not opposite to pRT_dir, we take the minus sign before vRT_norm
    double vRT_norm = -translationController.calculate(pRT_norm, 0.0);

    // compute rotation err, turn into angular velocity scalar
    double thetaRT = poseRobotTarget.getRotation().getRadians();
    double omegaRT = -rotationController.calculate(thetaRT, 0.0);

    // set limit
    double dCurr = finalDestinationPose.relativeTo(poseWorldRobot).getTranslation().getNorm(); // use final destination
    double vFar = ReefAimCommandParamsNT.translationVelocityMaxFar.getValue();
    double vNear = ReefAimCommandParamsNT.translationVelocityMaxNear.getValue();
    double dChange = ReefAimCommandParamsNT.translationParamsChangeDistance.getValue();
    double maxTranslationVelocityMps = dCurr > dChange ? vFar : vNear + dCurr / dChange * (vFar - vNear);
    vRT_norm = MathUtil.clamp(vRT_norm, 0.0, maxTranslationVelocityMps);
    Translation2d vRT = new Translation2d(vRT_norm, pRT_dir);

    // compose and run velocity with limit
    swerve.setSwerveLimit(
        SwerveLimit.builder()
            .maxLinearVelocity(MetersPerSecond.of(maxTranslationVelocityMps))
            .maxSkidAcceleration(MetersPerSecondPerSecond.of(ReefAimCommandParamsNT.translationAccelerationMax.getValue()))
            .maxAngularVelocity(DegreesPerSecond.of(ReefAimCommandParamsNT.rotationVelocityMax.getValue()))
            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(ReefAimCommandParamsNT.rotationAccelerationMax.getValue()))
            .build()
    );
    ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
    swerve.runTwist(VRT);

    // logging
    Logger.recordOutput(kTag + "/tagPose", tagPose);
    Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
    Logger.recordOutput(kTag + "/finalDestinationPose", finalDestinationPose);
    Logger.recordOutput(kTag + "/pRTNorm", pRT_norm);
    Logger.recordOutput(kTag + "/vRTNorm", vRT_norm);
    Logger.recordOutput(kTag + "/thetaRT", thetaRT);
    Logger.recordOutput(kTag + "/maxTranslationVelocityMps", maxTranslationVelocityMps);
  }

  @Override
  public boolean isFinished() {
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
    translationOnTarget = epsilonEquals(
        poseRobotTarget.getTranslation(), new Translation2d(),
        ReefAimCommandParamsNT.translationOnTargetToleranceMeter.getValue()
    );
    rotationOnTarget = epsilonEquals(
        poseRobotTarget.getRotation().getDegrees(),
        0.0,
        ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue()
    );

    translationStationary = epsilonEquals(
        velocityRobot.getTranslation(), new Translation2d(),
        ReefAimCommandParamsNT.translationOnTargetVelocityMetersPerSecond.getValue()
    );
    rotationStationary = epsilonEquals(
        velocityRobot.getRotation().getDegrees(), 0.0,
        ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue()
    );

    Logger.recordOutput(kTag + "/translationOnTarget", translationOnTarget);
    Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
    Logger.recordOutput(kTag + "/translationStationary", translationStationary);
    Logger.recordOutput(kTag + "/rotationStationary", rotationStationary);
    return translationOnTarget && rotationOnTarget && translationStationary && rotationStationary;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setSwerveLimitDefault();
    if (!interrupted) indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
    else indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @NTParameter(tableName = "Params/" + kTag)
  public static class ReefAimCommandParams {
    static final double translationKp = 6.5;
    static final double translationKi = 0.01;
    static final double translationKiZone = 0.5;
    static final double translationKd = 0.7;
    static final double translationVelocityMaxFar = 4.6;
    static final double translationVelocityMaxNear = 3.0;
    static final double translationParamsChangeDistance = 1.5;
    static final double translationAccelerationMax = 25.0;

    static final double rotationKp = 4.0;
    static final double rotationKi = 0.01;
    static final double rotationKiZone = 0.5;
    static final double rotationKd = 0.5;
    static final double rotationVelocityMax = 500.0;
    static final double rotationAccelerationMax = 1500.0;

    static final double translationOnTargetToleranceMeter = 0.02;
    static final double translationOnTargetVelocityMetersPerSecond = 0.3;
    static final double rotationOnTargetToleranceDegree = 3.5;
    static final double rotationOnTargetVelocityToleranceDegreesPerSecond = 30.0;
  }
}