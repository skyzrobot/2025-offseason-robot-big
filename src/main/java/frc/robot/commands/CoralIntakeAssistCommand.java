package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import lib.ntext.NTParameter;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

/**
 * Coral Intake Assist Drive Command
 * 
 * A drive command that provides normal joystick driving PLUS coral intake assist.
 * This command should replace the default swerve drive command when intaking.
 * 
 * Features:
 * - Normal joystick-based driving with all the same features as the default command
 * - Coral intake assist that adds perpendicular velocity toward detected corals
 * - Assist only activates when moving toward a coral target
 * 
 * Assist calculation:
 * - Calculates perpendicular distance from robot trajectory to coral
 * - Applies proportional control: assist_vel = |robot_to_coral_perp_to_vel| * kP
 * - Only provides assist if moving towards the coral
 */
public class CoralIntakeAssistCommand extends Command {
    private final Swerve swerve;
    private final PhotonVisionSubsystem photonVision;
    
    // Driver input suppliers
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier zSupplier;
    private final Supplier<Pose3d> poseDriveRobotSupplier;
    private final LinearVelocity translationDeadband;
    private final AngularVelocity rotationDeadband;
    
    // Joystick curves
    private final Function<Double, Double> translationJoystickCurve;
    private final Function<Double, Double> rotationJoystickCurve;
    
    public CoralIntakeAssistCommand(
            Swerve swerve, 
            PhotonVisionSubsystem photonVision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> poseDriveRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband) {
        this.swerve = swerve;
        this.photonVision = photonVision;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.poseDriveRobotSupplier = poseDriveRobotSupplier;
        this.translationDeadband = translationDeadband;
        this.rotationDeadband = rotationDeadband;
        
        // Use standard joystick curves
        this.translationJoystickCurve = (x) -> x; // Linear
        this.rotationJoystickCurve = (x) -> x * x * Math.signum(x); // Quadratic
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        System.out.println("CoralIntakeAssistCommand: Initialized");
    }
    
    @Override
    public void execute() {
        // Step 1: Process normal joystick input (copied from SwerveCommands.driveWithJoystick)
        SwerveLimit swerveLimit = swerve.getSwerveLimit();

        // Read from joystick
        double x = translationJoystickCurve.apply(xSupplier.getAsDouble());
        double y = translationJoystickCurve.apply(ySupplier.getAsDouble());
        double z = rotationJoystickCurve.apply(zSupplier.getAsDouble());

        // Compute linear velocity
        double vNorm = MathUtil.applyDeadband(
                Math.hypot(x, y) * swerveLimit.maxLinearVelocity().in(MetersPerSecond),
                translationDeadband.in(MetersPerSecond)
        );
        Rotation2d vDir = epsilonEquals(vNorm, 0.0) ? Rotation2d.kZero : new Rotation2d(x, y);
        Translation2d v = new Translation2d(vNorm, vDir);

        // Compute angular velocity
        double omegaNorm = MathUtil.applyDeadband(
                Math.abs(z) * swerveLimit.maxAngularVelocity().in(RadiansPerSecond),
                rotationDeadband.in(RadiansPerSecond)
        );
        double omegaDir = Math.signum(z);
        AngularVelocity omega = RadiansPerSecond.of(omegaNorm * omegaDir);

        // Compose to chassis speeds - base driver input
        ChassisSpeeds baseChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                v.getX(), v.getY(), omega.in(RadiansPerSecond),
                poseDriveRobotSupplier.get().getRotation().toRotation2d()
        );

        // Step 2: Calculate coral assist (if applicable)
        Translation2d assistVelocity = calculateCoralAssist(baseChassisSpeeds);
        
        // Step 3: Combine base movement with assist
        ChassisSpeeds finalChassisSpeeds = new ChassisSpeeds(
                baseChassisSpeeds.vxMetersPerSecond + assistVelocity.getX(),
                baseChassisSpeeds.vyMetersPerSecond + assistVelocity.getY(),
                baseChassisSpeeds.omegaRadiansPerSecond
        );

        // Apply the combined speeds
        swerve.runTwist(finalChassisSpeeds);
        
        // Logging
        Logger.recordOutput("CoralIntakeAssist/BaseSpeed",  baseChassisSpeeds);
        Logger.recordOutput("CoralIntakeAssist", finalChassisSpeeds);
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("CoralIntakeAssistCommand: Ended (interrupted: " + interrupted + ")");
    }
    
    @Override
    public boolean isFinished() {
        // This command should run continuously when conditions are met
        return false;
    }
    
    /**
     * Calculates coral assist velocity to add to base driver input
     * @param baseChassisSpeeds The chassis speeds from driver input
     * @return Translation2d assist velocity to add (x, y in meters/second)
     */
    private Translation2d calculateCoralAssist(ChassisSpeeds baseChassisSpeeds) {
        // Check if we have a coral target
        PhotonVisionSubsystem.RawDetection bestCoral = getBestCoral();
        if (bestCoral == null) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "No coral target");
            return new Translation2d();
        }
        
        // Only assist if robot is moving
        double robotSpeed = Math.hypot(baseChassisSpeeds.vxMetersPerSecond, baseChassisSpeeds.vyMetersPerSecond);
        if (robotSpeed < CoralIntakeAssistParamsNT.minRobotSpeed.getValue()) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "Robot not moving");
            Logger.recordOutput("CoralIntakeAssist/RobotSpeed", robotSpeed);
            return new Translation2d();
        }
        
        // Get robot and coral positions
        Pose2d robotPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        Pose3d coralPose3d = photonVision.projectTargetTo3D(bestCoral);
        Pose2d coralPose2d = coralPose3d.toPose2d();
        Translation2d coralPosition = robotPose.transformBy(new Transform2d(coralPose2d.getTranslation(), coralPose2d.getRotation())).getTranslation();
        
        // Only assist if moving towards coral
        Translation2d robotToCoralVector = coralPosition.minus(robotPose.getTranslation());
        Translation2d robotVelocityVector = new Translation2d(baseChassisSpeeds.vxMetersPerSecond, baseChassisSpeeds.vyMetersPerSecond);
        
        double dotProduct = robotVelocityVector.getX() * robotToCoralVector.getX() + 
                           robotVelocityVector.getY() * robotToCoralVector.getY();
        
        boolean movingTowardsCoral = dotProduct > 0;
        if (!movingTowardsCoral) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "Not moving towards coral");
            Logger.recordOutput("CoralIntakeAssist/DotProduct", dotProduct);
            return new Translation2d();
        }
        
        // Calculate assist velocity
        Translation2d assistVelocity = calculateAssistVelocity(robotPose, baseChassisSpeeds, coralPosition);
        
        // Apply limits
        double assistX = MathUtil.clamp(assistVelocity.getX(), 
            -CoralIntakeAssistParamsNT.maxAssistVelocity.getValue(), 
            CoralIntakeAssistParamsNT.maxAssistVelocity.getValue());
        double assistY = MathUtil.clamp(assistVelocity.getY(), 
            -CoralIntakeAssistParamsNT.maxAssistVelocity.getValue(), 
            CoralIntakeAssistParamsNT.maxAssistVelocity.getValue());
        
        // Additional logging
        Logger.recordOutput("CoralIntakeAssist/IsActive", true);
        Logger.recordOutput("CoralIntakeAssist/CoralDistance", robotToCoralVector.getNorm());
        Logger.recordOutput("CoralIntakeAssist/PerpendicularDistance", 
            calculatePerpendicularDistance(robotPose.getTranslation(), robotVelocityVector, coralPosition));
        Logger.recordOutput("CoralIntakeAssist/RobotSpeed", robotSpeed);
        Logger.recordOutput("CoralIntakeAssist/CoralYaw", bestCoral.yaw());
        Logger.recordOutput("CoralIntakeAssist/CoralPitch", bestCoral.pitch());
        
        System.out.println("CoralIntakeAssist: Active - Assist X: " + assistX + ", Y: " + assistY + 
                          ", Coral Distance: " + robotToCoralVector.getNorm());
        
        return new Translation2d(assistX, assistY);
    }

    /**
     * Calculates the assist velocity based on perpendicular distance to coral
     * Following the 2024 intake assist formula: assist_vel = |robot_to_coral_perp_to_vel| * kP
     */
    private Translation2d calculateAssistVelocity(Pose2d robotPose, ChassisSpeeds currentSpeeds, Translation2d coralPosition) {
        Translation2d robotVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        
        // Calculate perpendicular distance from robot trajectory to coral
        double perpDistance = calculatePerpendicularDistance(robotPose.getTranslation(), robotVelocity, coralPosition);
        
        // Calculate assist magnitude using proportional control
        double assistMagnitude = Math.abs(perpDistance) * CoralIntakeAssistParamsNT.assistKp.getValue();
        
        // Calculate direction perpendicular to robot velocity towards coral
        Translation2d perpDirection = calculatePerpendicularDirection(robotVelocity, robotPose.getTranslation(), coralPosition);
        
        return perpDirection.times(assistMagnitude);
    }
    
    /**
     * Calculates perpendicular distance from robot trajectory to coral position
     */
    private double calculatePerpendicularDistance(Translation2d robotPos, Translation2d robotVel, Translation2d coralPos) {
        if (robotVel.getNorm() < 0.01) return 0.0; // Avoid division by zero
        
        Translation2d robotToCoral = coralPos.minus(robotPos);
        Translation2d velUnit = robotVel.div(robotVel.getNorm());
        
        // Project robot-to-coral vector onto velocity direction
        double parallelComponent = robotToCoral.getX() * velUnit.getX() + robotToCoral.getY() * velUnit.getY();
        Translation2d parallelVector = velUnit.times(parallelComponent);
        
        // Perpendicular component is the remainder
        Translation2d perpVector = robotToCoral.minus(parallelVector);
        
        return perpVector.getNorm() * Math.signum(perpVector.getX() * velUnit.getY() - perpVector.getY() * velUnit.getX());
    }
    
    /**
     * Calculates the direction perpendicular to robot velocity towards the coral
     */
    private Translation2d calculatePerpendicularDirection(Translation2d robotVel, Translation2d robotPos, Translation2d coralPos) {
        if (robotVel.getNorm() < 0.01) {
            // If not moving, assist directly towards coral
            Translation2d robotToCoral = coralPos.minus(robotPos);
            return robotToCoral.getNorm() > 0.01 ? robotToCoral.div(robotToCoral.getNorm()) : new Translation2d();
        }
        
        Translation2d robotToCoral = coralPos.minus(robotPos);
        Translation2d velUnit = robotVel.div(robotVel.getNorm());
        
        // Get perpendicular direction (rotate velocity 90 degrees)
        Translation2d perpRight = new Translation2d(velUnit.getY(), -velUnit.getX());
        Translation2d perpLeft = new Translation2d(-velUnit.getY(), velUnit.getX());
        
        // Choose direction that points towards coral
        double dotRight = perpRight.getX() * robotToCoral.getX() + perpRight.getY() * robotToCoral.getY();
        double dotLeft = perpLeft.getX() * robotToCoral.getX() + perpLeft.getY() * robotToCoral.getY();
        
        return dotRight > dotLeft ? perpRight : perpLeft;
    }
    
    /**
     * Gets the best coral target for intake assist
     * Prioritizes closest to crosshair (smallest yaw angle)
     */
    private PhotonVisionSubsystem.RawDetection getBestCoral() {
        if (!photonVision.hasAnyTargets()) {
            return null;
        }
        
        // For intake assist, we want the coral closest to our crosshair
        // This makes the assist most predictable for the driver
        PhotonVisionSubsystem.RawDetection bestTarget = null;
        double smallestYaw = Double.MAX_VALUE;
        
        for (PhotonVisionSubsystem.RawDetection detection : photonVision.getAllRawDetections()) {
            // Only consider coral targets (not AprilTags)
            if (detection.isObject()) {
                double absYaw = Math.abs(detection.yaw());
                if (absYaw < smallestYaw && absYaw < CoralIntakeAssistParamsNT.maxYawDegrees.getValue()) {
                    smallestYaw = absYaw;
                    bestTarget = detection;
                }
            }
        }
        
        return bestTarget;
    }
    
    @NTParameter(tableName = "Params/Commands/CoralIntakeAssist")
    public static class CoralIntakeAssistParams {
        static final double assistKp = 0.8;
        static final double maxAssistVelocity = 1.5;
        static final double minRobotSpeed = 0.1;
        static final double maxYawDegrees = 30.0;
    }
} 