package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
  private SendableChooser<Command> autoSelector;
  private static AutoSelector instance;

  private AutoSelector() {
    autoSelector = new SendableChooser<>();
    SmartDashboard.putData("Auto Selector", autoSelector);
  }

  public static AutoSelector getInstance() {
    if (instance == null) {
      instance = new AutoSelector();
    }
    return instance;
  }

  public void registerAuto(String name, Command auto) {
    autoSelector.addOption(name, auto);
  }

  public Command getAutoCommand() {
    return autoSelector.getSelected();
  }
}
