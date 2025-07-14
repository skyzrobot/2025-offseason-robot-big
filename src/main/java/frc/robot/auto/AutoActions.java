package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.commands.aimSequences.ChaseCoralCommand;
import frc.robot.commands.aimSequences.ReefAimCommand;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ntext.NTParameter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Collections;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static frc.robot.commands.aimSequences.AimGoalSupplier.isInHexagonalReefDangerZone;
import static lib.ironpulse.math.MathTools.cross;
import static lib.ironpulse.math.MathTools.toAngle;

public class AutoActions {
  private static final Pose2d kLeftDecisionPoint = new Pose2d(
      new Translation2d(1.4, 6.8),
      Rotation2d.fromDegrees(144)
  );
  private static final Pose2d kLeftBackoff = new Pose2d(
      new Translation2d(4.0, 6.2),
      Rotation2d.fromDegrees(170)
  );
  private static final Pose2d kLeftEnd = new Pose2d(
      new Translation2d(2.50, 5.3),
      Rotation2d.fromDegrees(180)
  );
  private static final RotationTarget kLeftBackoffViewAngle = new RotationTarget(
      0.45, Rotation2d.fromDegrees(-10)
  );
  private static final RotationTarget kLeftDecisionPointViewAngle = new RotationTarget(
      1.0, Rotation2d.fromDegrees(-26)
  );


  private static final Pose2d kRightDecisionPoint = new Pose2d(
      new Translation2d(1.4, 1.2),
      Rotation2d.fromDegrees(-144)
  );
  private static final Pose2d kRightBackoff = new Pose2d(
      new Translation2d(4.0, 1.8),
      Rotation2d.fromDegrees(180.0)
  );
  private static final Pose2d kRightEnd = new Pose2d(
      new Translation2d(2.50, 2.7),
      Rotation2d.fromDegrees(180)
  );
  private static final RotationTarget kRightBackoffViewAngle = new RotationTarget(
      0.45, Rotation2d.fromDegrees(10)
  );
  private static final RotationTarget kRightDecisionPointViewAngle = new RotationTarget(
      1.0, Rotation2d.fromDegrees(26)
  );

  public static Swerve swerve;
  public static Superstructure superstructure;
  public static IndicatorSubsystem indicator;
  public static PhotonVisionSubsystem photon;

  public static void init(Swerve swerve, Superstructure superstructure, IndicatorSubsystem indicator, PhotonVisionSubsystem photon) {
    AutoActions.swerve = swerve;
    AutoActions.superstructure = superstructure;
    AutoActions.indicator = indicator;
    AutoActions.photon = photon;
  }

  public static Command intake() {
    return superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE);
  }

  public static Command intakeUtilComplete() {
    return superstructure
        .runGoal(AutoActions::determineIntakeState)
        .until(AutoActions::isIntakeComplete);
  }

  public static Command indicate(IndicatorIO.Patterns pattern) {
    return Commands.run(() -> indicator.setPattern(pattern));
  }

  public static Command chase() {
    return new ChaseCoralCommand(swerve).until(AutoActions::isInIntakeDangerZone);
  }

  public static Command chaseAndBackoff() {
    return Commands.sequence(
        chase().unless(AutoActions::isInIntakeDangerZone),
        driveToIntakePoint(false, false)
    ).until(superstructure::hasCoral);
  }

  public static Command setGoal(AimGoalSupplier.ReefFace face, boolean isRight, SuperstructureState level) {
    return Commands.runOnce(() -> {
      var dest = DestinationSupplier.getInstance();
      dest.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
      dest.setStateSetPoint(level);
      dest.updateBranch(isRight);
      AimGoalSupplier.setSelectedTarget(face);
    });
  }


  public static Command driveForwardBlind(double vx, double timeS) {
    return Commands.run(() -> {
          swerve.runTwist(new ChassisSpeeds(vx, 0.0, 0.0));
        }, swerve)
        .withTimeout(Seconds.of(timeS))
        .finallyDo(swerve::runStop);
  }

  public static Command driveToNearestTarget() {
    return new ReefAimCommand(swerve, indicator, false);
  }

  public static Command driveToSelectedTarget() {
    return new ReefAimCommand(swerve, indicator, true);
  }

  public static PathPlannerPath generatePath(List<Pose2d> waypoints, List<RotationTarget> rotationTargets, double maxVel, double maxAcc, double endVelMps) {
    PathConstraints constraints = new PathConstraints(
        maxVel, maxAcc,
        15.0, 40.0, 12.0
    );
    List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(waypoints);
    Pose2d lastPose = waypoints.get(waypoints.size() - 1);
    GoalEndState endState = new GoalEndState(endVelMps, lastPose.getRotation());
    return new PathPlannerPath(
        pts,
        rotationTargets,
        Collections.emptyList(),
        Collections.emptyList(),
        Collections.emptyList(),
        constraints,
        null,
        endState,
        false
    );
  }

  public static PathPlannerPath generatePath(List<Pose2d> waypoints, List<RotationTarget> rotationTargets, double endVelMps) {
    return generatePath(waypoints, rotationTargets, 4.5, 15.0, endVelMps);
  }

  public static Command driveToIntakePoint(boolean isLeft, boolean shouldBackoff) {
    return swerve.defer(() -> {
      Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
      Pose2d backoff = AllianceFlipUtil.apply(isLeft ? kLeftBackoff : kRightBackoff);
      Pose2d decision = AllianceFlipUtil.apply(isLeft ? kLeftDecisionPoint : kRightDecisionPoint);
      RotationTarget backoffAngle = isLeft ? kLeftBackoffViewAngle : kRightBackoffViewAngle;
      RotationTarget decisionAngle = isLeft ? kLeftDecisionPointViewAngle : kRightDecisionPointViewAngle;

      List<Pose2d> waypoints = shouldBackoff
          ? List.of(current, backoff, decision)
          : List.of(current, decision);
      List<RotationTarget> rotationTargets = shouldBackoff
          ? List.of(backoffAngle, decisionAngle)
          : List.of(decisionAngle);

      PathPlannerPath path = generatePath(waypoints, rotationTargets, 3.5, 5.0, 0.0);
      return Commands.deadline(
          followPath(path),
          applySwerveLimit().repeatedly()
      );
    });
  }

  public static Command driveToEndPoint(boolean isLeft) {
    return swerve.defer(() -> {
      Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
      Pose2d end = AllianceFlipUtil.apply(isLeft ? kLeftEnd : kRightEnd);

      List<Pose2d> waypoints = List.of(current, end);

      PathPlannerPath path = generatePath(waypoints, Collections.emptyList(), 0.0);
      return followPath(path);
    });
  }

  public static Command followPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
        swerve::getChassisSpeeds,
        (vel, ff) -> {
          swerve.runTwist(vel);
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.5, 0.0, 0.1),
            new PIDConstants(4.5, 0.0, 0.1),
            RobotConstants.LOOPER_DT
        ),
        RobotConstants.AUTO_ROBOT_CONFIG,
        () -> false, // do not flip in command, flip done by user before passing
        swerve
    ).beforeStarting(
        () -> Logger.recordOutput("Temp/Traj", path.getPathPoses().toArray(new Pose2d[0]))
    );
  }

  public static Command prepare() {
    return Commands
        .runOnce(() -> DestinationSupplier.getInstance()
            .setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING))
        .andThen(
            Commands.parallel(
                Commands.waitUntil(AutoActions::isSafeToRaise)
                    .onlyIf(() -> (DestinationSupplier
                        .getInstance().getPreState() == SuperstructureState.L4))
                    .andThen(superstructure
                        .runGoal(() -> DestinationSupplier
                            .getInstance()
                            .getPreState())
                        .until(superstructure::atGoal))));

  }

  public static Command shoot() {
    return superstructure
        .runGoal(
            () -> DestinationSupplier
                .getInstance()
                .getShootState())
        .until(() -> !superstructure.hasCoral());
  }

  public static Command takeAlgae() {
    var destinationSupplier = DestinationSupplier.getInstance();
    return Commands
        .runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING))
        .andThen(
            Commands.parallel(
                superstructure
                    .runGoal(() -> DestinationSupplier
                        .getInstance()
                        .getPreState())
                    .until(superstructure::hasAlgae)
            ),
            superstructure
                .runGoal(destinationSupplier::getPreState)
                .until(() -> !isInHexagonalReefDangerZone(
                    RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d()))
                .finallyDo(() -> System.out.println("done"))
        );
  }

  public static Command reset() {
    return SwerveCommands.resetAngle(swerve, new Rotation2d())
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            })).ignoringDisable(true);
  }

  public static Command resetOnPose(Pose2d pose) {
    var resetPose = new Pose3d(AllianceFlipUtil.apply(pose));

    return SwerveCommands.reset(swerve, resetPose)
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            }))
        .onlyIf(Robot::isSimulation)
        .ignoringDisable(true);
  }

  public static Command resetOnPathStart(PathPlannerPath path) {
    var realPath = AllianceFlipUtil.shouldFlip() ? path.flipPath() : path;

    return SwerveCommands.reset(swerve, new Pose3d(realPath.getStartingHolonomicPose().get()))
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            }))
        .onlyIf(() -> realPath.getStartingHolonomicPose().isPresent())
        .ignoringDisable(true);
  }

  public static Command limitSwerve(
      double maxVelocityMps, double maxAccelerationMps2,
      double maxAngularVelDegps, double maxAngularAccelerationDegps2
  ) {
    return Commands.runOnce(() -> swerve.setSwerveLimit(
        SwerveLimit.builder()
            .maxLinearVelocity(MetersPerSecond.of(maxVelocityMps))
            .maxSkidAcceleration(MetersPerSecondPerSecond.of(maxAccelerationMps2))
            .maxAngularVelocity(DegreesPerSecond.of(maxAngularVelDegps))
            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(maxAngularAccelerationDegps2))
            .build()
    ));
  }

  public static Command unlimitSwerve() {
    return Commands.runOnce(swerve::setSwerveLimitDefault);
  }

  public static Command indicateEnd() {
    return Commands.print("Auto Ended!");
  }

  public static Command applySwerveLimit() {
    return Commands.runOnce(() -> {
      double elevatorHeight = superstructure.getElevatorPosition();
      double startLimitHeight = AutoParamsNT.TrajectoryLimitStartHeight.getValue();
      double maxVel = AutoParamsNT.TrajectoryMaxLinVelMps.getValue();
      double maxAcc = AutoParamsNT.TrajectoryMaxLinAccelMps2.getValue();
      double limitedVel, limitedAcc;
      if (elevatorHeight > startLimitHeight) {
        double maxExtension = RobotConstants.ElevatorConstants.MAX_EXTENSION_METERS.get();
        double minVel = AutoParamsNT.TrajectoryLimitedLinVelMps.getValue();
        double minAcc = AutoParamsNT.TrajectoryLimitedLinAccelMps2.getValue();
        double range = maxExtension - startLimitHeight;
        double delta = elevatorHeight - startLimitHeight;
        double k = MathUtil.clamp(delta / range, 0, 1); // k \in [0, 1]
        limitedVel = maxVel * (1.0 - k) + minVel * k;
        limitedAcc = maxAcc * (1.0 - k) + minAcc * k;
      } else {
        limitedVel = maxVel;
        limitedAcc = maxAcc;
      }
      swerve.setSwerveLimit(
          SwerveLimit.builder()
              .maxLinearVelocity(MetersPerSecond.of(limitedVel))
              .maxSkidAcceleration(MetersPerSecondPerSecond.of(limitedAcc))
              .maxAngularVelocity(DegreesPerSecond.of(
                  AutoParamsNT.TrajectoryMaxAngVelDegps.getValue()
              ))
              .maxAngularAcceleration(DegreesPerSecondPerSecond.of(
                  AutoParamsNT.TrajectoryMaxAngAccelDegps2.getValue()
              ))
              .build()
      );
    });
  }

  // ----------------------------------- Helpers ------------------------------------------------
  private static boolean isSafeToRaise() {
    return DestinationSupplier.isSafeToRaise(
        RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
        DestinationSupplier.getInstance().getCurrentBranch()
    );
  }

  private static boolean isInReefDangerZone() {
    Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    return AimGoalSupplier.isInHexagonalReefDangerZone(pose);
  }

  private static boolean isIntakeComplete() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    if (hasAlgae) {
      // When we have algae, we need both algae AND indexed coral
      return superstructure.hasAlgae() && superstructure.hasIndexedCoral();
    } else if (!inDangerZone) {
      // When not in danger zone, we just need coral
      return superstructure.hasCoral();
    } else {
      // When in danger zone (without algae), we need indexed coral
      return superstructure.hasIndexedCoral();
    }
  }

  private static SuperstructureState determineIntakeState() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    // If we have algae OR we're in danger zone, use indexed intake
    // Otherwise, use ground intake for safety
    if (hasAlgae || inDangerZone) {
      return SuperstructureState.CORAL_INDEXED_INTAKE;
    } else {
      return SuperstructureState.CORAL_GROUND_INTAKE;
    }
  }


  public static boolean isInIntakeDangerZone() {
    // get robot position in field-flipped coordinates
    var p = AllianceFlipUtil
        .apply(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d())
        .getTranslation();

    // define the two line endpoints and direction vectors
    var rRight = new Translation2d(0.0, AutoParamsNT.RightTriangleY.getValue());
    var vRight = new Translation2d(
        AutoParamsNT.RightTriangleX.getValue(),
        0.0
    ).minus(rRight);

    var rLeft = new Translation2d(0.0, AutoParamsNT.LeftTriangleY.getValue());
    var vLeft = new Translation2d(
        AutoParamsNT.LeftTriangleX.getValue(),
        FieldConstants.fieldWidth
    ).minus(rLeft);

    double xBottom = AutoParamsNT.BoundaryOffset.getValue();
    var pRelRight = p.minus(rRight);
    double crossRight = cross(pRelRight, vRight);
    var pRelLeft = p.minus(rLeft);
    double crossLeft = cross(pRelLeft, vLeft);

    boolean beyondBottom = p.getX() < xBottom;
    boolean beyondRightLine = crossRight > 0;
    boolean beyondLeftLine = crossLeft < 0;
    boolean outOfYBounds = p.getY() < AutoParamsNT.BoundaryOffset.getValue() || p.getY() > FieldConstants.fieldWidth - AutoParamsNT.BoundaryOffset.getValue();

    return beyondRightLine || beyondLeftLine || beyondBottom || outOfYBounds;
  }

  public static boolean isCoralInSight() {
    var coralTarget = RobotStateRecorder.getNearestCoral();
    if (coralTarget.isEmpty()) return false;
    var posWorldCoral = coralTarget.get().getTranslation();
    var poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();

    var dirRobotCoral = toAngle(posWorldCoral.minus(poseWorldRobot.getTranslation()));
    return Math.abs(dirRobotCoral.getDegrees()) < AutoParamsNT.CoralInSightDegs.getValue();
  }

  public static boolean isControlCoral() {
    return superstructure.hasCoral() || superstructure.hasIndexedCoral();
  }

  @NTParameter(tableName = "Params/Auto")
  public static class AutoParams {
    static final double TrajectoryMaxLinVelMps = 4.5;
    static final double TrajectoryMaxLinAccelMps2 = 14.0;
    static final double TrajectoryMaxAngVelDegps = 700.0;
    static final double TrajectoryMaxAngAccelDegps2 = 1800.0;

    static final double TrajectoryLimitedLinVelMps = 2.5;
    static final double TrajectoryLimitedLinAccelMps2 = 6.0;
    static final double TrajectoryLimitStartHeight = 0.65;

    static final double CoralInSightDegs = 80.0;

    static final double RightTriangleX = 2.5;
    static final double RightTriangleY = 1.7;
    static final double LeftTriangleX = 2.5;
    static final double LeftTriangleY = 6.3;
    static final double BoundaryOffset = 0.6;
  }

}
