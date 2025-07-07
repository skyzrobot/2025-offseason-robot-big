package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

public class BlocklessEitherCommand extends Command {
  private final Command onTrue;
  private final Command onFalse;
  private final BooleanSupplier condition;
  private boolean conditionWhenSelect;
  public BlocklessEitherCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
    this.onTrue = onTrue;
    this.onFalse = onFalse;
    this.condition = condition;
  }

  @Override
  public void initialize() {
    conditionWhenSelect = condition.getAsBoolean();
    if (conditionWhenSelect) onTrue.schedule();
    else onFalse.schedule();
  }

  @Override
  public boolean isFinished() {
    return conditionWhenSelect ? onTrue.isFinished() : onFalse.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (conditionWhenSelect) onTrue.cancel();
    else onFalse.cancel();
  }
}
