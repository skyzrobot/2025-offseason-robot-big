package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.commands.aimSequences.SuperCycleCommand;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;

import static edu.wpi.first.units.Units.*;

public class AutoActions {
  private static Swerve swerve;
  private static Superstructure superstructure;
  private static IndicatorSubsystem indicator;

  private static PathPlannerPath testPath;

  public static void init(Swerve swerve, Superstructure superstructure, IndicatorSubsystem indicator) {
    AutoActions.swerve = swerve;
    AutoActions.superstructure = superstructure;
    AutoActions.indicator = indicator;

    try {
      AutoActions.testPath = PathPlannerPath.fromPathFile("B-I3");
    } catch (Exception e) {
      Logging.error("AutoActions", "Error when loading trajectory! %s", e.getMessage());
    }
  }

  public static Command intakeUtilComplete() {
    return superstructure
        .runGoal(AutoActions::determineIntakeState)
        .until(AutoActions::isIntakeComplete);
  }

  public static Command intake() {
    return superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE);
  }

  public static Command score(boolean isRightBranch, SuperstructureState state) {
    var destinationSupplier = DestinationSupplier.getInstance();
    return Commands.sequence(
        Commands.runOnce(() -> destinationSupplier.updateBranch(isRightBranch)),
        Commands.runOnce(() -> destinationSupplier.setStateSetPoint(state)),
        new SuperCycleCommand(swerve, superstructure, indicator).finallyDo(
            () -> createDangerZoneExitCommand().schedule()
        )
    ).onlyIf(() -> superstructure.hasCoral());
  }

  public static Command reset() {
    return SwerveCommands.resetAngle(swerve, new Rotation2d())
        .alongWith(Commands.runOnce(
            () -> {
              RobotStateRecorder.getInstance().resetTransform(
                  TransformRecorder.kFrameWorld,
                  TransformRecorder.kFrameRobot
              );
            }));
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

  public static Command followTrajectory_Test() {
    return followPath(testPath);
  }


  // ----------------------------------- Helpers ------------------------------------------------

  /**
   * Helper method to check if robot is in the hexagonal reef danger zone
   *
   * @return true if robot is in danger zone
   */
  private static boolean isInReefDangerZone() {
    Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    return AimGoalSupplier.isInHexagonalReefDangerZone(pose);
  }

  /**
   * Determines the appropriate intake state based on current conditions
   *
   * @return SuperstructureState for intake operation
   */
  private static SuperstructureState determineIntakeState() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    System.out.println("Intake State Decision - HasAlgae: " + hasAlgae + ", InDangerZone: " + inDangerZone);

    // If we have algae OR we're in danger zone, use indexed intake
    // Otherwise, use ground intake for safety
    if (hasAlgae || inDangerZone) {
      return SuperstructureState.CORAL_INDEXED_INTAKE;
    } else {
      return SuperstructureState.CORAL_GROUND_INTAKE;
    }
  }

  /**
   * Determines if the intake operation is complete based on current conditions
   *
   * @return true if intake is complete
   */
  private static boolean isIntakeComplete() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    if (hasAlgae) {
      // When we have algae, we need both algae AND indexed coral
      return superstructure.hasAlgae() && superstructure.indexedCoral();
    } else if (!inDangerZone) {
      // When not in danger zone, we just need coral
      return superstructure.hasCoral();
    } else {
      // When in danger zone (without algae), we need indexed coral
      return superstructure.indexedCoral();
    }
  }

  /**
   * Creates a command to move the superstructure to algae prestate until robot exits danger zone
   * This is used after SuperCycleCommand completes to ensure safe exit from reef area
   *
   * @return Command that continues to algae prestate until out of danger zone
   */
  private static Command createDangerZoneExitCommand() {
    var destinationSupplier = DestinationSupplier.getInstance();
    return Commands.sequence(
        // Set game piece to algae intaking to get correct prestate
        Commands.runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING)),
        // Continue running to algae prestate until out of danger zone
        superstructure
            .runGoal(() -> destinationSupplier.getPreState())
            .until(() -> !isInReefDangerZone())
    ).onlyIf(AutoActions::isInReefDangerZone); // Only run if actually in danger zone
  }

  /**
   * Follow a pathplanner path.
   *
   * @param path pathplanner path.
   * @return command to follow path.
   */
  public static Command followPath(PathPlannerPath path) {
    var chassisSpeedCurrent = swerve.getChassisSpeeds();
    var poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent();
    var trajectory = path.generateTrajectory(
        chassisSpeedCurrent,
        poseWorldRobot.getRotation().toRotation2d(),
        RobotConstants.AUTO_ROBOT_CONFIG
    );

    return SwerveCommands.followPathPlannerTrajectory(
        swerve, trajectory,
        RobotStateRecorder::getPoseWorldRobotCurrent,
        new PIDController(3.5, 0.0, 0.0),
        new PIDController(5.0, 0.0, 0.3),
        Meters.of(0.05),
        Degrees.of(5.0),
        event -> {}
    );
  }
}
