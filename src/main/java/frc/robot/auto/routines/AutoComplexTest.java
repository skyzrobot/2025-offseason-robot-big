package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutine;
import lib.ironpulse.command.DecisionTree;

import java.util.Collections;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoComplexTest extends AutoRoutine {
  private int tryCounter = 0;

  public AutoComplexTest() {
    super("Complex Auto");
  }

  @Override
  public Command getAutoCommand() {
    var action1 = print("starting").alongWith(waitSeconds(1.0));
    var action2 = defer(() -> print("currently counting " + tryCounter++), Collections.emptySet()).alongWith(waitSeconds(1.0));
    var action3 = print("ending");

    var tree = new DecisionTree(action1).addAlwaysTrueDecision(action1, action2).addDecision(action2, action2, () -> tryCounter < 3).addDecision(action2, action3, () -> tryCounter >= 3);

    return tree.toCommand();
  }
}
