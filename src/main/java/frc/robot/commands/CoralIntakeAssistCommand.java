package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.utils.CoralRecorder;
import lib.ntext.NTParameter;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

/**
 * Coral Intake Assist Drive Command
 * 
 * Provides normal joystick driving with intelligent coral intake assistance.
 * This command should replace the default swerve drive command when intaking corals.
 * 
 * Features:
 * - Normal joystick-based driving with standard deadband and input curves
 * - Coral intake assist that adds perpendicular velocity toward aligned corals
 * - Assist only activates when moving toward coral within configurable angle threshold
 * - Uses CoralRecorder to track coral positions consistently over time
 * 
 * How it works:
 * 1. Gets most aligned coral from CoralRecorder
 * 2. Checks if driver input direction is within angle threshold of coral
 * 3. Calculates perpendicular distance from trajectory to coral  
 * 4. Applies proportional assist velocity perpendicular to driver input
 * 5. Combines driver input + assist for final robot movement
 */
public class CoralIntakeAssistCommand extends Command {
    private final Swerve swerve;
    
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
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> poseDriveRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband) {
        
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.poseDriveRobotSupplier = poseDriveRobotSupplier;
        this.translationDeadband = translationDeadband;
        this.rotationDeadband = rotationDeadband;
        
        // Standard joystick input curves
        this.translationJoystickCurve = x -> x; // Linear for translation
        this.rotationJoystickCurve = x -> x * x * Math.signum(x); // Quadratic for rotation
        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Step 1: Process driver joystick input
        ChassisSpeeds baseChassisSpeeds = calculateBaseChassisSpeedsFromJoystick();

        // Step 2: Calculate coral intake assist
        Translation2d assistVelocity = calculateCoralAssist(baseChassisSpeeds);
        
        // Step 3: Combine driver input with assist
        ChassisSpeeds finalChassisSpeeds = new ChassisSpeeds(
                baseChassisSpeeds.vxMetersPerSecond + assistVelocity.getX(),
                baseChassisSpeeds.vyMetersPerSecond + assistVelocity.getY(),
                baseChassisSpeeds.omegaRadiansPerSecond
        );

        // Apply combined movement to swerve drive
        swerve.runTwist(finalChassisSpeeds);
        
        // Log speeds for debugging
        Logger.recordOutput("CoralIntakeAssist/BaseSpeed", baseChassisSpeeds);
        Logger.recordOutput("CoralIntakeAssist/FinalSpeed", finalChassisSpeeds);
        Logger.recordOutput("CoralIntakeAssist/AssistVel", assistVelocity);
    }
    
    /**
     * Calculates base chassis speeds from joystick input using standard drive logic.
     */
    private ChassisSpeeds calculateBaseChassisSpeedsFromJoystick() {
        SwerveLimit swerveLimit = swerve.getSwerveLimit();

        // Apply input curves to joystick values
        double x = translationJoystickCurve.apply(xSupplier.getAsDouble());
        double y = translationJoystickCurve.apply(ySupplier.getAsDouble());
        double z = rotationJoystickCurve.apply(zSupplier.getAsDouble());

        // Calculate linear velocity with deadband
        double vNorm = MathUtil.applyDeadband(
                Math.hypot(x, y) * swerveLimit.maxLinearVelocity().in(MetersPerSecond),
                translationDeadband.in(MetersPerSecond)
        );
        Rotation2d vDir = epsilonEquals(vNorm, 0.0) ? Rotation2d.kZero : new Rotation2d(x, y);
        Translation2d velocity = new Translation2d(vNorm, vDir);

        // Calculate angular velocity with deadband
        double omegaNorm = MathUtil.applyDeadband(
                Math.abs(z) * swerveLimit.maxAngularVelocity().in(RadiansPerSecond),
                rotationDeadband.in(RadiansPerSecond)
        );
        AngularVelocity omega = RadiansPerSecond.of(omegaNorm * Math.signum(z));

        // Convert to field-relative chassis speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(), velocity.getY(), omega.in(RadiansPerSecond),
                poseDriveRobotSupplier.get().getRotation().toRotation2d()
        );
    }
    
    @Override
    public boolean isFinished() {
        // This command should run continuously when conditions are met
        return false;
    }
    
    /**
     * Calculates coral assist velocity to add to base driver input.
     * Uses CoralRecorder to get the most aligned coral target and calculates assist in world frame.
     * 
     * @param baseChassisSpeeds The chassis speeds from driver input
     * @return Translation2d assist velocity to add in robot frame (x, y in meters/second)
     */
    private Translation2d calculateCoralAssist(ChassisSpeeds baseChassisSpeeds) {
        // Get robot state`
        Pose2d robotPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        Translation2d driverVelocityRobotFrame = new Translation2d(baseChassisSpeeds.vxMetersPerSecond, baseChassisSpeeds.vyMetersPerSecond);
        Translation2d driverVelocityWorldFrame = driverVelocityRobotFrame.rotateBy(robotPose.getRotation());
        
        // Check minimum speed requirement first
        double driverSpeed = driverVelocityWorldFrame.getNorm();
        if (driverSpeed < CoralIntakeAssistParamsNT.minRobotSpeed.getValue()) {
            logInactiveAssist("Driver speed too low", driverSpeed);
            return new Translation2d();
        }
        
        // Get nearest coral within driving direction cone
        Optional<CoralRecorder.CoralInfo> coralOpt = getNearestCoralInDrivingDirection(robotPose, driverVelocityWorldFrame);
        if (coralOpt.isEmpty()) {
            logInactiveAssist("No coral in driving direction", driverSpeed);
            return new Translation2d();
        }
        
        Translation2d coralPosition = coralOpt.get().getTranslation();
        Translation2d robotToCoral = coralPosition.minus(robotPose.getTranslation());
        double angleToCoralDegrees = calculateAngleBetweenVectors(driverVelocityWorldFrame, robotToCoral);
        
        // Calculate and apply assist
        Translation2d assistWorldFrame = calculateAssistVelocityWorld(robotPose.getTranslation(), driverVelocityWorldFrame, coralPosition);
        Translation2d assistClamped = clampAssistVelocity(assistWorldFrame);
        Translation2d assistRobotFrame = assistClamped.rotateBy(robotPose.getRotation().unaryMinus());
        
        // Log active assist
        logActiveAssist(robotPose, driverVelocityWorldFrame, coralPosition, robotToCoral, angleToCoralDegrees, driverSpeed, assistClamped, assistRobotFrame);
        
        return assistRobotFrame;
    }
    
    /**
     * Calculates the angle between two vectors in degrees.
     */
    private double calculateAngleBetweenVectors(Translation2d vector1, Translation2d vector2) {
        double magnitude1 = vector1.getNorm();
        double magnitude2 = vector2.getNorm();
        
        if (magnitude1 < 0.01 || magnitude2 < 0.01) {
            return 180.0; // Return large angle if either vector is too small
        }
        
        double dotProduct = vector1.getX() * vector2.getX() + vector1.getY() * vector2.getY();
        double cosAngle = MathUtil.clamp(dotProduct / (magnitude1 * magnitude2), -1.0, 1.0);
        return Math.toDegrees(Math.acos(cosAngle));
    }
    
    /**
     * Clamps the assist velocity to maximum allowed values.
     */
    private Translation2d clampAssistVelocity(Translation2d assistVelocity) {
        double maxVel = CoralIntakeAssistParamsNT.maxAssistVelocity.getValue();
        double assistX = MathUtil.clamp(assistVelocity.getX(), -maxVel, maxVel);
        double assistY = MathUtil.clamp(assistVelocity.getY(), -maxVel, maxVel);
        return new Translation2d(assistX, assistY);
    }
    
    /**
     * Logs when assist is inactive with reason.
     */
    private void logInactiveAssist(String reason, double driverSpeed) {
        Logger.recordOutput("CoralIntakeAssist/IsActive", false);
        Logger.recordOutput("CoralIntakeAssist/Reason", reason);
    }
    
    /**
     * Logs when assist is active with all relevant data.
     */
    private void logActiveAssist(Pose2d robotPose, Translation2d driverVelocity, Translation2d coralPosition, 
                                Translation2d robotToCoral, double angleToCoralDegrees, double driverSpeed,
                                Translation2d assistWorldFrame, Translation2d assistRobotFrame) {
        Logger.recordOutput("CoralIntakeAssist/BestCoral", new Pose2d(coralPosition,Rotation2d.kZero));
        Logger.recordOutput("CoralIntakeAssist/IsActive", true);
        Logger.recordOutput("CoralIntakeAssist/AngleToCoralDegrees", angleToCoralDegrees);
        Logger.recordOutput("CoralIntakeAssist/PerpendicularDistance", 
            calculatePerpendicularDistance(robotPose.getTranslation(), driverVelocity, coralPosition));
        Logger.recordOutput("CoralIntakeAssist/AssistWorldFrame", assistWorldFrame);
    }

    /**
     * Gets the nearest coral within the driving direction cone.
     * This creates a cone in the direction the driver is commanding and finds the nearest coral within it.
     */
    private Optional<CoralRecorder.CoralInfo> getNearestCoralInDrivingDirection(Pose2d robotPose, Translation2d driverVelocityWorldFrame) {
        if (driverVelocityWorldFrame.getNorm() < 0.01) {
            return Optional.empty(); // No direction to define cone
        }
        
        // Get all available corals
        Optional<CoralRecorder.CoralInfo> nearestCoral = RobotStateRecorder.getNearestCoral();
        Optional<CoralRecorder.CoralInfo> mostAlignedCoral = RobotStateRecorder.getMostInDirectionCoral();
        
        // For now, we'll check both corals (if available) and return the nearest one within the cone
        // This is a simplified implementation - ideally we'd get all corals and filter them
        
        double coneAngleDegrees = CoralIntakeAssistParamsNT.maxAngleTowardsCoralDegrees.getValue();
        Optional<CoralRecorder.CoralInfo> bestCoral = Optional.empty();
        double nearestDistance = Double.MAX_VALUE;
        
        // Check nearest coral
        if (nearestCoral.isPresent()) {
            Translation2d robotToCoral = nearestCoral.get().getTranslation().minus(robotPose.getTranslation());
            double angle = calculateAngleBetweenVectors(driverVelocityWorldFrame, robotToCoral);
            
            if (angle <= coneAngleDegrees) {
                double distance = robotToCoral.getNorm();
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    bestCoral = nearestCoral;
                }
            }
        }
        
        // Check most aligned coral
        if (mostAlignedCoral.isPresent()) {
            Translation2d robotToCoral = mostAlignedCoral.get().getTranslation().minus(robotPose.getTranslation());
            double angle = calculateAngleBetweenVectors(driverVelocityWorldFrame, robotToCoral);
            
            if (angle <= coneAngleDegrees) {
                double distance = robotToCoral.getNorm();
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    bestCoral = mostAlignedCoral;
                }
            }
        }
        
        return bestCoral;
    }
    
    /**
     * Calculates the assist velocity based on perpendicular distance to coral in world frame.
     * Uses proportional control: assist_velocity = |perpendicular_distance| * kP
     */
    private Translation2d calculateAssistVelocityWorld(Translation2d robotPosition, Translation2d robotVelocity, Translation2d coralPosition) {
        double perpDistance = calculatePerpendicularDistance(robotPosition, robotVelocity, coralPosition);
        double assistMagnitude = Math.abs(perpDistance) * CoralIntakeAssistParamsNT.assistKp.getValue();
        Translation2d perpDirection = calculatePerpendicularDirection(robotVelocity, robotPosition, coralPosition);
        return perpDirection.times(assistMagnitude);
    }
    
    /**
     * Calculates signed perpendicular distance from robot trajectory to coral position.
     * Positive = coral to left of trajectory, Negative = coral to right of trajectory
     */
    private double calculatePerpendicularDistance(Translation2d robotPos, Translation2d robotVel, Translation2d coralPos) {
        if (robotVel.getNorm() < 0.01) return 0.0;
        
        Translation2d robotToCoral = coralPos.minus(robotPos);
        Translation2d velUnit = robotVel.div(robotVel.getNorm());
        
        // Project robot-to-coral vector onto velocity direction
        double parallelComponent = robotToCoral.getX() * velUnit.getX() + robotToCoral.getY() * velUnit.getY();
        Translation2d perpVector = robotToCoral.minus(velUnit.times(parallelComponent));
        
        // Use cross product to determine sign (left vs right of trajectory)
        double crossProduct = robotToCoral.getX() * velUnit.getY() - robotToCoral.getY() * velUnit.getX();
        return perpVector.getNorm() * Math.signum(crossProduct);
    }
    
    /**
     * Calculates the unit vector perpendicular to robot velocity pointing towards the coral.
     */
    private Translation2d calculatePerpendicularDirection(Translation2d robotVel, Translation2d robotPos, Translation2d coralPos) {
        if (robotVel.getNorm() < 0.01) {
            // If not moving, point directly towards coral
            Translation2d robotToCoral = coralPos.minus(robotPos);
            return robotToCoral.getNorm() > 0.01 ? robotToCoral.div(robotToCoral.getNorm()) : new Translation2d();
        }
        
        Translation2d robotToCoral = coralPos.minus(robotPos);
        Translation2d velUnit = robotVel.div(robotVel.getNorm());
        
        // Calculate perpendicular component of robot-to-coral vector
        double parallelComponent = robotToCoral.getX() * velUnit.getX() + robotToCoral.getY() * velUnit.getY();
        Translation2d perpVector = robotToCoral.minus(velUnit.times(parallelComponent));
        
        // Return unit vector in perpendicular direction
        double perpNorm = perpVector.getNorm();
        return perpNorm > 0.01 ? perpVector.div(perpNorm) : new Translation2d();
    }
    
    @NTParameter(tableName = "Params/Commands/CoralIntakeAssist")
    public static class CoralIntakeAssistParams {
        static final double assistKp = 1.0;                        // Proportional gain for assist velocity
        static final double maxAssistVelocity = 3.0;               // Maximum assist velocity (m/s)
        static final double minRobotSpeed = 0.2;                   // Minimum robot speed to activate assist (m/s)
        static final double maxAngleTowardsCoralDegrees = 55.0;
        }    // Maximum angle to consider "moving towards" coral (degrees) -     
} 