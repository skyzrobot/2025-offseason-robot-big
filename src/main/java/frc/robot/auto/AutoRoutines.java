package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutoRoutines {
  public static Command testAuto()  {
    return AutoActions.followTrajectory_Test();
  }
}
