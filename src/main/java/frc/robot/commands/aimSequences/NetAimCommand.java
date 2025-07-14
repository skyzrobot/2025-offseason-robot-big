package frc.robot.commands.aimSequences;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.indicator.IndicatorIO;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;

import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class NetAimCommand extends Command {
  private final static String kTag = "Commands/NetAimCommand";
  private final Swerve swerve;
  private final DoubleSupplier yVelocitySupplier;
  private ProfiledPIDController xController;
  private PIDController rotationController;
  private Pose2d poseWorldRobot, poseWorldTarget;

  public NetAimCommand(Swerve swerve, DoubleSupplier yVelocitySupplier) {
    this.swerve = swerve;
    this.yVelocitySupplier = yVelocitySupplier;

    xController = new ProfiledPIDController(
        NetAimCommandParamsNT.xKp.getValue(),
        NetAimCommandParamsNT.xKi.getValue(),
        NetAimCommandParamsNT.xKd.getValue(),
        new TrapezoidProfile.Constraints(
            NetAimCommandParamsNT.translationVelocityMaxFar.getValue(),
            NetAimCommandParamsNT.translationAccelerationMax.getValue()));
    rotationController = new PIDController(
        NetAimCommandParamsNT.rotationKp.getValue(),
        NetAimCommandParamsNT.rotationKi.getValue(),
        NetAimCommandParamsNT.rotationKd.getValue());
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // tuning
    if (RobotConstants.TUNING) {
      xController.setP(NetAimCommandParamsNT.xKp.getValue());
      xController.setI(NetAimCommandParamsNT.xKi.getValue());
      xController.setIZone(NetAimCommandParamsNT.xKiZone.getValue());
      xController.setD(NetAimCommandParamsNT.xKd.getValue());

      rotationController.setP(NetAimCommandParamsNT.rotationKp.getValue());
      rotationController.setI(NetAimCommandParamsNT.rotationKi.getValue());
      rotationController.setIZone(NetAimCommandParamsNT.rotationKiZone.getValue());
      rotationController.setD(NetAimCommandParamsNT.rotationKd.getValue());
    }

    // calculate destination
    poseWorldTarget = AimGoalSupplier.getFinalNetTarget();
    var velocityWorldRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

    // PID init with field-relative velocities
    rotationController.enableContinuousInput(0, Math.PI * 2);
    xController.setTolerance(0.05, 0.15);
    xController.reset(poseWorldRobot.getTranslation().getX(), velocityWorldRobot.getX());
    rotationController.reset();
  }

  @Override
  public void execute() {
    poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);

    double xCurr = poseWorldRobot.getTranslation().getX();
    double xFinal = poseWorldTarget.getTranslation().getX();

    double vx = xController.calculate(xCurr, xFinal);
    double vy = yVelocitySupplier.getAsDouble();
    double thetaRT = poseRobotTarget.getRotation().getRadians();
    double omegaRT = -rotationController.calculate(thetaRT, 0.0);

    // compute limit
    double dCurr = Math.abs(xCurr - xFinal); // use final destination
    double vFar = NetAimCommandParamsNT.translationVelocityMaxFar.getValue();
    double vNear = NetAimCommandParamsNT.translationVelocityMaxNear.getValue();
    double dChange = NetAimCommandParamsNT.translationParamsChangeDistance.getValue();
    double maxTranslationVelocityMps = dCurr > dChange ? vFar : vNear + dCurr / dChange * (vFar - vNear);

    // compose and run velocity with limit
    swerve.setSwerveLimit(
        SwerveLimit.builder()
            .maxLinearVelocity(MetersPerSecond.of(maxTranslationVelocityMps))
            .maxSkidAcceleration(
                MetersPerSecondPerSecond.of(NetAimCommandParamsNT.translationAccelerationMax.getValue()))
            .maxAngularVelocity(DegreesPerSecond.of(NetAimCommandParamsNT.rotationVelocityMax.getValue()))
            .maxAngularAcceleration(
                DegreesPerSecondPerSecond.of(NetAimCommandParamsNT.rotationAccelerationMax.getValue()))
            .build());
    ChassisSpeeds VRT = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRT, poseWorldRobot.getRotation());
    swerve.runTwist(VRT);

    // logging
    Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
    Logger.recordOutput(kTag + "/thetaRT", thetaRT);
    Logger.recordOutput(kTag + "/maxTranslationVelocityMps", maxTranslationVelocityMps);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished!");
    swerve.setSwerveLimitDefault();
    swerve.runStop();
  }

  @NTParameter(tableName = "Params/" + kTag)
  public static class NetAimCommandParams {
    static final double xKp = 3.5;
    static final double xKi = 0.0;
    static final double xKiZone = 0.5;
    static final double xKd = 0.1;
    static final double translationVelocityMaxFar = 3.0;
    static final double translationVelocityMaxNear = 2.0;
    static final double translationParamsChangeDistance = 2.0;
    static final double translationAccelerationMax = 8.0;

    static final double rotationKp = 4.0;
    static final double rotationKi = 0.01;
    static final double rotationKiZone = 0.5;
    static final double rotationKd = 0.3;
    static final double rotationVelocityMax = 360.0;
    static final double rotationAccelerationMax = 1200.0;
  }
}
