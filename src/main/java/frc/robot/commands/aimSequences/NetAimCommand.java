package frc.robot.commands.aimSequences;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.indicator.IndicatorIO;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class NetAimCommand extends Command {
  private final static String kTag = "Commands/NetAimCommand";
  private final Swerve swerve;
  private final DoubleSupplier yVelocitySupplier;
  private PIDController xController;
  private PIDController rotationController;
  private Pose2d poseWorldRobot, poseWorldTarget;

  public NetAimCommand(Swerve swerve, DoubleSupplier yVelocitySupplier) {
    this.swerve = swerve;
    this.yVelocitySupplier = yVelocitySupplier;

    xController = new PIDController(
        NetAimCommandParamsNT.xKp.getValue(),
        NetAimCommandParamsNT.xKi.getValue(),
        NetAimCommandParamsNT.xKd.getValue()
    );
    rotationController = new PIDController(
        NetAimCommandParamsNT.rotationKp.getValue(),
        NetAimCommandParamsNT.rotationKi.getValue(),
        NetAimCommandParamsNT.rotationKd.getValue()
    );
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // tuning
    if (RobotConstants.TUNING) {
      xController.setP(ReefAimCommandParamsNT.translationKp.getValue());
      xController.setI(ReefAimCommandParamsNT.translationKi.getValue());
      xController.setIZone(ReefAimCommandParamsNT.translationKiZone.getValue());
      xController.setD(ReefAimCommandParamsNT.translationKd.getValue());
      xController.setTolerance(
          ReefAimCommandParamsNT.translationOnTargetToleranceMeter.getValue(),
          ReefAimCommandParamsNT.translationOnTargetVelocityMetersPerSecond.getValue()
      );

      rotationController.setP(ReefAimCommandParamsNT.rotationKp.getValue());
      rotationController.setI(ReefAimCommandParamsNT.rotationKi.getValue());
      rotationController.setIZone(ReefAimCommandParamsNT.rotationKiZone.getValue());
      rotationController.setD(ReefAimCommandParamsNT.rotationKd.getValue());
      rotationController.setTolerance(
          ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue() / 180.0f * Math.PI,
          ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue() / 180.0f * Math.PI
      );

      Logging.info(
          kTag, "Aiming Params: <Drive> Kp = %.2f, Ki = %.2f, Kd = %.2f" +
              "<Rotation> Kp = %.2f, Ki = %.2f, Kd = %.2f",
          ReefAimCommandParamsNT.translationKp.getValue(),
          ReefAimCommandParamsNT.translationKi.getValue(),
          ReefAimCommandParamsNT.translationKd.getValue(),
          ReefAimCommandParamsNT.rotationKp.getValue(),
          ReefAimCommandParamsNT.rotationKi.getValue(),
          ReefAimCommandParamsNT.rotationKd.getValue()
      );
    }

    // calculate destination
    poseWorldTarget = AimGoalSupplier.getFinalNetTarget();

    // PID init with field-relative velocities
    rotationController.enableContinuousInput(0, Math.PI * 2);
    xController.reset();
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
            .maxSkidAcceleration(MetersPerSecondPerSecond.of(ReefAimCommandParamsNT.translationAccelerationMax.getValue()))
            .maxAngularVelocity(DegreesPerSecond.of(ReefAimCommandParamsNT.rotationVelocityMax.getValue()))
            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(ReefAimCommandParamsNT.rotationAccelerationMax.getValue()))
            .build()
    );
    ChassisSpeeds VRT = new ChassisSpeeds(vx, vy, omegaRT);
    swerve.runTwist(VRT);

    // logging
    Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
    Logger.recordOutput(kTag + "/thetaRT", thetaRT);
    Logger.recordOutput(kTag + "/maxTranslationVelocityMps", maxTranslationVelocityMps);
  }

  @Override
  public boolean isFinished() {
    return false; // stop on operator command
  }


  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished!");
    swerve.setSwerveLimitDefault();
    swerve.runStop();
  }

  @NTParameter(tableName = "Params/" + kTag)
  public static class NetAimCommandParams {
    static final double xKp = 5.5;
    static final double xKi = 0.01;
    static final double xKiZone = 0.5;
    static final double xKd = 0.7;
    static final double translationVelocityMaxFar = 4.6;
    static final double translationVelocityMaxNear = 2.5;
    static final double translationParamsChangeDistance = 1.5;
    static final double translationAccelerationMax = 25.0;

    static final double rotationKp = 4.0;
    static final double rotationKi = 0.01;
    static final double rotationKiZone = 0.5;
    static final double rotationKd = 0.5;
    static final double rotationVelocityMax = 500.0;
    static final double rotationAccelerationMax = 1500.0;
  }
}
