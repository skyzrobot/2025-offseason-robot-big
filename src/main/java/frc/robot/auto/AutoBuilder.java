package frc.robot.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lib.ironpulse.command.DecisionTree;
import lib.ironpulse.utils.Logging;
import lombok.Setter;

import java.util.Set;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoBuilder {
  private static AutoBuilder instance;
  private int idx = 0;
  private boolean firstTimeToIntake = true;
  private boolean hasSeenCoral = false;
  // ------- Configs -------
  @Setter
  private AutoConfig config = new AutoConfig();

  private AutoBuilder() {
  }

  public static AutoBuilder getInstance() {
    if (instance == null) {
      instance = new AutoBuilder();
    }
    return instance;
  }

  private Pair<AimGoalSupplier.ReefFace, Boolean> decodeLocation(
      AutoConfig.AutoType type,
      AutoConfig.ScoringLocation location
  ) {
    if (type == AutoConfig.AutoType.LeftRoutine) {
      switch (location) {
        case FarLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.FarLeftTilt, false);
        }
        case FarRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.FarLeftTilt, true);
        }
        case NearLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearLeftTilt, false);
        }
        case NearRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearLeftTilt, true);
        }
        case FlatLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearFlat, false);
        }
        case FlatRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearFlat, true);
        }
        default -> throw new IllegalArgumentException("Unknown location: " + location);
      }
    } else if (type == AutoConfig.AutoType.RightRoutine) {
      switch (location) {
        case FarLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.FarRightTilt, false);
        }
        case FarRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.FarRightTilt, true);
        }
        case NearLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearRightTilt, false);
        }
        case NearRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearRightTilt, true);
        }
        case FlatLeft -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearFlat, false);
        }
        case FlatRight -> {
          return Pair.of(AimGoalSupplier.ReefFace.NearFlat, true);
        }
        default -> throw new IllegalArgumentException("Unknown location: " + location);
      }
    } else {
      throw new IllegalArgumentException("Unknown auto type: " + type);
    }
  }

  private SuperstructureState decodeScoringLevel(AutoConfig.ScoringLevel level) {
    return switch (level) {
      case L4 -> SuperstructureState.L4;
      case L2 -> SuperstructureState.L2;
      default -> SuperstructureState.L3;
    };
  }

  private Command setGoalBasedOnIdx() {
    Logging.info("Auto", "Currently on coral idx %d.", idx);

    var scoringTarget = config.getScoringTarget(idx);
    if (scoringTarget.isEmpty()) {
      return Commands.none();
    }

    var loc = scoringTarget.get().getFirst();
    var level = scoringTarget.get().getSecond();
    var decoded = decodeLocation(config.getAutoType(), loc);

    return setGoal(
        decoded.getFirst(), decoded.getSecond(), decodeScoringLevel(level)
    );
  }


  // ------- Auto Actions -------
  private void reset() {
    idx = 0;
    hasSeenCoral = false;
    RobotStateRecorder.setCoralFilterRegion(null);
    if (Robot.isSimulation()) {
      var pose = config.getAutoType() == AutoConfig.AutoType.LeftRoutine ? kLeftStartPose : kRightStartPose;
      AutoActions.resetOnPose(pose).schedule();
    }
    superstructure.startAuto();
  }

  private void advanceCoralIdx() {
    Logging.info("Auto", "Advancing coral idx from %d to %d.", idx, idx + 1);
    idx++;
  }

  public Command getAutoCommand() {
    var start = runOnce(this::reset);

    var scorePreload = sequence(
        print("Scoring Preload"),
        defer(this::setGoalBasedOnIdx, Set.of()),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot(),
        runOnce(this::advanceCoralIdx)
    ).finallyDo(() -> hasSeenCoral = false);

    var getCoral = print("Getting Coral").andThen(
        defer(() -> {
          boolean isLeft = config.getAutoType() == AutoConfig.AutoType.LeftRoutine;
          return deadline(
              sequence(
                  driveToIntakePoint(isLeft, firstTimeToIntake).until(AutoActions::isCoralInSight),
                  chase().onlyIf(AutoActions::isCoralInSight)
              ).until(() -> AutoActions.isInIntakeDangerZone() || hasSeenCoral || AutoActions.hasCoralAtEE()),
              parallel(
//                  forceZero().onlyIf(() -> idx == 1),
                  intake()
//                  sequence(
//                      waitUntil(() -> superstructure.getState() == SuperstructureState.CORAL_GROUND_INTAKE && superstructure.poseAtGoal()),
//                      superstructure.runZero()
//                  )
              )
          );
        }, Set.of(swerve, superstructure))
    ).finallyDo(
        () -> {
          swerve.runStop();
          firstTimeToIntake = false;
        }
    );

    var driveToBackoffPoint = sequence(
        print("Driving To Backoff Point"),
        deadline(
            swerve.defer(() -> {
                  boolean isLeft = config.getAutoType() == AutoConfig.AutoType.LeftRoutine;
                  return driveToBackoffPoint(isLeft);
                }
            ),
            intake()
        )
    ).unless(() -> hasSeenCoral || AutoActions.hasCoralAtEE());

    var score = sequence(
        print("Scoring"),
        defer(this::setGoalBasedOnIdx, Set.of()),
        sequence(
            parallel(
                driveToSelectedTarget(),
                sequence(
                    intake()
                        .onlyIf(() -> !AutoActions.hasCoralAtEE())
                        .until(AutoActions::hasCoralAtEE),
                    print("Coral At EE, Prepare"),
                    prepare()
                )
            ),
            sequence(
                shoot(),
                runOnce(this::advanceCoralIdx)
            )
        )
    ).finallyDo(() -> hasSeenCoral = false);

    var end = sequence(
        print("Ending"),
        AutoActions.takeAlgae().onlyIf(() -> config.getAutoType() != AutoConfig.AutoType.DoNothing)
    );

    var tree = new DecisionTree();
    tree.addRoot(start);
    tree.addDecision(score, end, () -> idx >= config.getCoralCount());
    tree.addAlwaysTrueDecision(start, scorePreload);
    tree.addAlwaysTrueDecision(scorePreload, getCoral);

    // branch 1: has coral, score
    tree.addDecision(getCoral, score, () -> hasSeenCoral || hasCoralAtEE());

    // branch 2: does not have coral, drive back off and retry
    tree.addDecision(getCoral, driveToBackoffPoint, () -> !hasCoralAtEE() && !hasSeenCoral);
    tree.addDecision(driveToBackoffPoint, getCoral, () -> !hasCoralAtEE() && !hasSeenCoral);
    tree.addDecision(driveToBackoffPoint, score, () -> hasSeenCoral || hasCoralAtEE());

    // repeat until all corals scored
    tree.addDecision(score, getCoral, () -> idx < config.getCoralCount());
    tree.addDecision(score, end, () -> idx >= config.getCoralCount());

    return deadline(
        tree.toCommand(),
        run(() -> {
          if (!hasSeenCoral && superstructure.hasIndexedCoral()) {
            hasSeenCoral = true;
          }
        })
    );
  }

}
