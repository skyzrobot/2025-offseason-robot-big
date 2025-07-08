package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class AutoActions {
  private static Swerve swerve;
  private static Superstructure superstructure;
  private static IndicatorSubsystem indicator;

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

//  public static Command limitSwerve(double maxVelocityMps, double maxAccelerationMps2) {
//
//  }

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
}
