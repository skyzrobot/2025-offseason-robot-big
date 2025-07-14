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
    var action1 = print("action1");
    var action2 = print("action2");
    var action3 = print("action3");

    var tree = new DecisionTree(action1)
        .addAlwaysTrueDecision(action1, action2)
        .addAlwaysTrueDecision(action2, action3);

    return tree.toCommand();
  }
}
