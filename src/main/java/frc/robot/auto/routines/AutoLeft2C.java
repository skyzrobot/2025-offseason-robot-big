package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lib.ironpulse.command.DecisionTree;
import lib.ironpulse.utils.Logging;

import java.util.Set;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoLeft2C extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, FieldConstants.fieldWidth - 0.50),
      Rotation2d.kZero
  );

  private int idxCoral = 0;
  private final Timer indexTimer = new Timer();

  public AutoLeft2C() {
    super("Left2C");
  }

  private void reset() {
    idxCoral = 0;
  }
  private void advanceCoralIdx() {
    Logging.info("Auto", "Advancing coral cound from %d to %d.", idxCoral, idxCoral + 1);
    idxCoral++;
  }

  private Command setGoalBasedOnIdx() {
    Logging.info("Auto", "Currently on coral idx %d.", idxCoral);
    indexTimer.reset();
    indexTimer.start();

    return switch (idxCoral) {
      case 0 -> setGoal(AimGoalSupplier.ReefFace.FarLeftTilt, false, SuperstructureState.L4);
      case 1 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L4);
      case 2 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L4);
      case 3 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L3);
      case 4 -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L3);
      default -> setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L2);
    };
  }

  @Override
  public Command getAutoCommand() {
    var start = Commands.runOnce(this::reset);

    var scorePreload = sequence(
        defer(this::setGoalBasedOnIdx, Set.of()),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot(),
        Commands.runOnce(this::advanceCoralIdx)
    );

    var getCoral = defer(
        () -> {
          boolean backoff = idxCoral == 1; // only back off for the first coral
          return deadline(
              sequence(
                  deadline(
                      driveToIntakePoint(true, backoff),
                      indicate(IndicatorIO.Patterns.INTAKE)
                  ).until(AutoActions::isCoralInSight),
                  deadline(
                      chase(),
                      indicate(IndicatorIO.Patterns.ASSISTED_INTAKE)
                  ).onlyIf(AutoActions::isCoralInSight)
              ).until(() -> AutoActions.isInIntakeDangerZone() || AutoActions.isControlCoral()),
              intake()
          );
        }, Set.of(swerve, superstructure)
    );

    var scoreWithConsider = sequence(
        defer(this::setGoalBasedOnIdx, Set.of()),
        sequence(
            parallel(
                driveToSelectedTarget(),
                prepare()
            ),
            sequence(
                shoot(),
                Commands.runOnce(this::advanceCoralIdx)
            ).onlyIf(AutoActions::isControlCoral)
        ).unless(() -> indexTimer.hasElapsed(1.0) && !AutoActions.isControlCoral())
    );

    var end = AutoActions.takeAlgae();

    var tree = new DecisionTree();
    tree.addRoot(start);
    tree.addAlwaysTrueDecision(start, scorePreload);
    tree.addAlwaysTrueDecision(scorePreload, getCoral);
    tree.addAlwaysTrueDecision(getCoral, scoreWithConsider);
    tree.addDecision(scoreWithConsider, getCoral, () -> this.idxCoral < 2);
    tree.addDecision(scoreWithConsider, end, () -> this.idxCoral >= 2);

    return tree.toCommand();
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }
}
