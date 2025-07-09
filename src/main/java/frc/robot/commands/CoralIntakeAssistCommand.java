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
 * A drive command that provides normal joystick driving PLUS coral intake assist.
 * This command should replace the default swerve drive command when intaking.
 * Uses CoralRecorder to track coral positions consistently over time.
 * 
 * Features:
 * - Normal joystick-based driving with all the same features as the default command
 * - Coral intake assist that adds perpendicular velocity toward the most aligned coral
 * - Assist only activates when moving toward the most aligned coral within angle threshold
 * - All calculations done in world frame for consistency
 * 
 * Assist calculation:
 * - Uses CoralRecorder's most aligned coral target
 * - Checks if robot velocity angle to coral is within configurable threshold
 * - Calculates perpendicular distance from robot trajectory to coral in world frame
 * - Applies proportional control: assist_vel = |robot_to_coral_perp_to_vel| * kP
 * - Converts final result to chassis speeds for swerve
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
     * Uses CoralRecorder to get the most aligned coral target and calculates assist in world frame
     * @param baseChassisSpeeds The chassis speeds from driver input
     * @return Translation2d assist velocity to add in world frame (x, y in meters/second)
     */
    private Translation2d calculateCoralAssist(ChassisSpeeds baseChassisSpeeds) {
        // Get robot pose and velocity in world frame
        Pose2d robotWorldPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        Pose2d robotWorldVelocity = RobotStateRecorder.getVelocityWorldRobotCurrent();
        
        // Get the most aligned coral from CoralRecorder
        Optional<CoralRecorder.CoralInfo> mostAlignedCoralOpt = getMostAlignedCoral(robotWorldPose);
        if (mostAlignedCoralOpt.isEmpty()) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "No aligned coral target");
            return new Translation2d();
        }
        
        CoralRecorder.CoralInfo mostAlignedCoral = mostAlignedCoralOpt.get();
        Translation2d coralWorldPosition = mostAlignedCoral.getTranslation();
        
        // Only assist if robot is moving
        Translation2d robotWorldVelocityVector = robotWorldVelocity.getTranslation();
        double robotSpeed = robotWorldVelocityVector.getNorm();
        if (robotSpeed < CoralIntakeAssistParams.minRobotSpeed) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "Robot not moving");
            Logger.recordOutput("CoralIntakeAssist/RobotSpeed", robotSpeed);
            return new Translation2d();
        }
        
        // Check if moving towards the most aligned coral within angle threshold
        Translation2d robotToCoralVector = coralWorldPosition.minus(robotWorldPose.getTranslation());
        double dotProduct = robotWorldVelocityVector.getX() * robotToCoralVector.getX() + 
                           robotWorldVelocityVector.getY() * robotToCoralVector.getY();
        
        // Calculate angle threshold check: cos(maxAngle) * |velocity| * |robotToCoral|
        double requiredDotProduct = Math.cos(Math.toRadians(CoralIntakeAssistParams.maxAngleTowardsCoralDegrees)) 
                                   * robotWorldVelocityVector.getNorm() 
                                   * robotToCoralVector.getNorm();
        
        boolean movingTowardsCoral = dotProduct >= requiredDotProduct;
        
        // Calculate actual angle for logging (with safety checks)
        double velocityMagnitude = robotWorldVelocityVector.getNorm();
        double robotToCoralMagnitude = robotToCoralVector.getNorm();
        double actualAngleDegrees = 0.0;
        
        if (velocityMagnitude > 0.01 && robotToCoralMagnitude > 0.01) {
            double cosAngle = MathUtil.clamp(dotProduct / (velocityMagnitude * robotToCoralMagnitude), -1.0, 1.0);
            actualAngleDegrees = Math.toDegrees(Math.acos(cosAngle));
        }
        
        // Detailed logging for debugging
        Logger.recordOutput("CoralIntakeAssist/CoralWorldPosition", coralWorldPosition);
        Logger.recordOutput("CoralIntakeAssist/DotProduct", dotProduct);
        Logger.recordOutput("CoralIntakeAssist/RequiredDotProduct", requiredDotProduct);
        Logger.recordOutput("CoralIntakeAssist/MovingTowardsCoral", movingTowardsCoral);
        
        if (!movingTowardsCoral) {
            Logger.recordOutput("CoralIntakeAssist/IsActive", false);
            Logger.recordOutput("CoralIntakeAssist/Reason", "Not moving towards coral within angle threshold");
            System.out.println("CoralIntakeAssist: Not active - Angle to coral: " + actualAngleDegrees + 
                             "°, Threshold: " + CoralIntakeAssistParams.maxAngleTowardsCoralDegrees + "°");
            return new Translation2d();
        }
        
        // Calculate assist velocity in world frame
        Translation2d assistVelocityWorld = calculateAssistVelocityWorld(
            robotWorldPose.getTranslation(), 
            robotWorldVelocityVector, 
            coralWorldPosition
        );
        
        // Apply limits
        double assistX = MathUtil.clamp(assistVelocityWorld.getX(), 
            -CoralIntakeAssistParams.maxAssistVelocity, 
            CoralIntakeAssistParams.maxAssistVelocity);
        double assistY = MathUtil.clamp(assistVelocityWorld.getY(), 
            -CoralIntakeAssistParams.maxAssistVelocity, 
            CoralIntakeAssistParams.maxAssistVelocity);
        
        Translation2d finalAssistWorld = new Translation2d(assistX, assistY);
        
        // Convert world frame assist velocity to robot frame for chassis speeds
        Translation2d assistRobotFrame = finalAssistWorld.rotateBy(robotWorldPose.getRotation().unaryMinus());
        
        // Additional logging
        Logger.recordOutput("CoralIntakeAssist/IsActive", true);
        Logger.recordOutput("CoralIntakeAssist/CoralDistance", robotToCoralVector.getNorm());
        Logger.recordOutput("CoralIntakeAssist/PerpendicularDistance", 
            calculatePerpendicularDistance(robotWorldPose.getTranslation(), robotWorldVelocityVector, coralWorldPosition));
        Logger.recordOutput("CoralIntakeAssist/RobotSpeed", robotSpeed);
        Logger.recordOutput("CoralIntakeAssist/AssistWorldFrame", finalAssistWorld);
        Logger.recordOutput("CoralIntakeAssist/AssistRobotFrame", assistRobotFrame);
        
        
        return assistRobotFrame;
    }

    /**
     * Gets the most aligned coral from CoralRecorder
     * @param robotPose Current robot pose in world frame (unused, method uses current pose internally)
     * @return Optional containing the most aligned coral, or empty if none available
     */
    private Optional<CoralRecorder.CoralInfo> getMostAlignedCoral(Pose2d robotPose) {
        return RobotStateRecorder.getMostInDirectionCoral();
    }
    
    /**
     * Calculates the assist velocity based on perpendicular distance to coral in world frame
     * Following the 2024 intake assist formula: assist_vel = |robot_to_coral_perp_to_vel| * kP
     * @param robotPosition Robot position in world frame
     * @param robotVelocity Robot velocity in world frame
     * @param coralPosition Coral position in world frame
     * @return Assist velocity in world frame
     */
    private Translation2d calculateAssistVelocityWorld(Translation2d robotPosition, Translation2d robotVelocity, Translation2d coralPosition) {
        // Calculate perpendicular distance from robot trajectory to coral
        double perpDistance = calculatePerpendicularDistance(robotPosition, robotVelocity, coralPosition);
        
                // Calculate assist magnitude using proportional control
        double assistMagnitude = Math.abs(perpDistance) * CoralIntakeAssistParams.assistKp;
        
        // Calculate direction perpendicular to robot velocity towards coral
        Translation2d perpDirection = calculatePerpendicularDirection(robotVelocity, robotPosition, coralPosition);
        
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
     * Simplified approach: directly calculate the perpendicular component of robot-to-coral vector
     */
    private Translation2d calculatePerpendicularDirection(Translation2d robotVel, Translation2d robotPos, Translation2d coralPos) {
        if (robotVel.getNorm() < 0.01) {
            // If not moving, assist directly towards coral
            Translation2d robotToCoral = coralPos.minus(robotPos);
            return robotToCoral.getNorm() > 0.01 ? robotToCoral.div(robotToCoral.getNorm()) : new Translation2d();
        }
        
        Translation2d robotToCoral = coralPos.minus(robotPos);
        Translation2d velUnit = robotVel.div(robotVel.getNorm());
        
        // Project robot-to-coral onto velocity direction
        double parallelComponent = robotToCoral.getX() * velUnit.getX() + robotToCoral.getY() * velUnit.getY();
        Translation2d parallelVector = velUnit.times(parallelComponent);
        
        // Perpendicular component points toward coral
        Translation2d perpVector = robotToCoral.minus(parallelVector);
        
        // Return unit vector in perpendicular direction
        double perpNorm = perpVector.getNorm();
        return perpNorm > 0.01 ? perpVector.div(perpNorm) : new Translation2d();
    }
    
    @NTParameter(tableName = "Params/Commands/CoralIntakeAssist")
    public static class CoralIntakeAssistParams {
        static final double assistKp = 0.8;                        // Proportional gain for assist velocity
        static final double maxAssistVelocity = 1.5;               // Maximum assist velocity (m/s)
        static final double minRobotSpeed = 0.1;                   // Minimum robot speed to activate assist (m/s)              // Maximum yaw angle for coral selection (degrees)
        static final double maxAngleTowardsCoralDegrees = 45.0;    // Maximum angle to consider "moving towards" coral (degrees)
    }
} 