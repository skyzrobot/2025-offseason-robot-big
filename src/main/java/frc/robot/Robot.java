package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.utils.LoggedTracer;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import static frc.robot.RobotConstants.DriverCamera;
import static frc.robot.RobotConstants.LOOPER_DT;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    super(LOOPER_DT);
  }

  @Override
  public void robotInit() {
    if (!RobotConstants.useReplay) {
      // logger initialization
      Logger.addDataReceiver(new NT4Publisher());
      //Logger.addDataReceiver(new WPILOGWriter());
    } else {
      // Replaying a log, set up replay source
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), AdvantageScopeOpenBehavior.ALWAYS));
    }


    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.start();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // early-stage initialization
    DriverStation.silenceJoystickConnectionWarning(true);
    PowerDistribution PDP = new PowerDistribution();
    PDP.clearStickyFaults();
    PDP.close();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    PhoenixUtils.refreshAll();
    CommandScheduler.getInstance().run();

    LoggedTracer.record("Commands");
    LoggedTracer.record("RobotPeriodic");
    if (RobotConstants.TUNING)
      NTParameterRegistry.refresh();
    robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {
    System.out.println(AimGoalSupplier.isInHexagonalReefDangerZone(new Pose2d()));
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    try {
      //todo: add autonomous command
      autonomousCommand = AutoSelector.getInstance().getAutoCommand();
//      autonomousCommand = Commands.none();
    } catch (Exception e) {
      System.out.println("Autonomous command failed: " + e);
      e.printStackTrace();
      autonomousCommand = null;
    }

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {

  }
}
