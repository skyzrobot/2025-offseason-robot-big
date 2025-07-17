package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef;
import frc.robot.auto.AutoBuilder;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.utils.LoggedTracer;
import lib.ironpulse.math.filter.ButterworthFilter;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import javax.sound.sampled.Line;
import java.lang.reflect.Field;

import static frc.robot.RobotConstants.DriverCamera;
import static frc.robot.RobotConstants.LOOPER_DT;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public static PowerDistribution powerDistribution;
  public static LinearFilter voltageFilter = LinearFilter.singlePoleIIR(1.0 / (2.0 * Math.PI * 100), LOOPER_DT);
  public static LinearFilter currentFilter = LinearFilter.singlePoleIIR(1.0 / (2.0 * Math.PI * 100), LOOPER_DT);
  public static LinearFilter powerFilter = LinearFilter.singlePoleIIR(1.0 / (2.0 * Math.PI * 100), LOOPER_DT);

  public Robot() {
    super(LOOPER_DT);
    powerDistribution = new PowerDistribution();
  }

  @Override
  public void robotInit() {
    if (!RobotConstants.useReplay) {
      // logger initialization
      Logger.addDataReceiver(new NT4Publisher());
      Logger.addDataReceiver(new WPILOGWriter());
    } else {
      // Replaying a log, set up replay source
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), AdvantageScopeOpenBehavior.ALWAYS));
    }

    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.start();

    // early-stage initialization
    DriverStation.silenceJoystickConnectionWarning(true);

    // config watchdog
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(0.2);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(0.2);

    powerDistribution.clearStickyFaults();
    robotContainer = new RobotContainer();

    // warm-up path-following
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    PhoenixUtils.refreshAll();
    CommandScheduler.getInstance().run();
    NTParameterRegistry.refresh();

    robotContainer.robotPeriodic();
    AutoSelector.getInstance().updateAlerts();

    double voltage = powerDistribution.getVoltage();
    double current = powerDistribution.getTotalCurrent();
    double power = powerDistribution.getTotalPower();

    Logger.recordOutput("Power/Current", current);
    Logger.recordOutput("Power/Voltage", voltage);
    Logger.recordOutput("Power/Power", power);

    Logger.recordOutput("Power/CurrentFiltered", currentFilter.calculate(current));
    Logger.recordOutput("Power/VoltageFiltered", voltageFilter.calculate(voltage));
    Logger.recordOutput("Power/PowerFiltered", powerFilter.calculate(power));
  }

  @Override
  public void disabledInit() {
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
      AutoBuilder.getInstance().setConfig(AutoSelector.getInstance().getAutoConfig());
      autonomousCommand = AutoBuilder.getInstance().getAutoCommand();
    } catch (Exception e) {
      System.out.println("Autonomous command failed: " + e);
      e.printStackTrace();
      autonomousCommand = null;
    }

    if (autonomousCommand != null) autonomousCommand.schedule();
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
