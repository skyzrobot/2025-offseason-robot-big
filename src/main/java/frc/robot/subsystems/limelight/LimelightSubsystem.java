package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.limelight.LimelightIO.PoseEstimate;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.LoggedTracer;
import lombok.Getter;

import org.frcteam6941.localization.Localizer;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.LimelightConstants.*;

public class LimelightSubsystem extends SubsystemBase {
    private final Map<String, LimelightIO> limelightIOs;
    private final Map<String, LimelightIOInputsAutoLogged> limelightInputs;
    @Getter
    public Optional<PoseEstimate[]> estimatedPose;

    private final Localizer swerveLocalizer = Swerve.getInstance().getLocalizer();
    private boolean useMegaTag2 = false;

    public LimelightSubsystem(Map<String, LimelightIO> limelightIOs) {
        this.limelightIOs = limelightIOs;

        limelightInputs = new HashMap<>();
        this.limelightIOs.forEach((key, value) -> limelightInputs.put(key, new LimelightIOInputsAutoLogged()));
    }

    public static boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
        if (poseEstimate == null) {
            return true;
        }

        // Angular velocity is too high to have accurate vision
        if (gyroRate.compareTo(RobotConstants.SwerveConstants.maxAngularRate) > 0) {
            return true;
        }
        //TODO: verify this condition whether usable

        // No tags :<
        if (poseEstimate.tagCount() == 0) {
            return true;
        }

        // 1 Tag with a large area
        if (poseEstimate.tagCount() == 1 && poseEstimate.avgTagArea() > AREA_THRESHOLD) {
            // TODO: BUG: area threshold is wayyyy to small the tag area is 0-100% of original tag
            return false;
            // 2 tags or more
        } else return poseEstimate.tagCount() <= 1;
    }

    public PoseEstimate[] getLastPoseEstimates() {
        List<PoseEstimate> poseEstimates = new ArrayList<>();
        limelightInputs.forEach((key, input) -> {
            if (input != null && input.poseBlue != null) {
                poseEstimates.add(input.poseBlue);
            }
        });
        return poseEstimates.toArray(new PoseEstimate[0]);
    }

    /**
     * Gets the minimum ambiguity value from all visible AprilTags across all limelights.
     * Returns Double.MAX_VALUE if no tags are visible.
     * 
     * @return The minimum ambiguity value, or Double.MAX_VALUE if no tags visible
     */
    public double getMinimumAmbiguity() {
        double minAmbiguity = Double.MAX_VALUE;
        
        for (LimelightIOInputsAutoLogged input : limelightInputs.values()) {
            if (input != null && input.poseBlue != null && input.poseBlue.rawFiducials() != null) {
                for (LimelightIO.RawFiducial fiducial : input.poseBlue.rawFiducials()) {
                    if (fiducial != null) {
                        minAmbiguity = Math.min(minAmbiguity, fiducial.ambiguity());
                    }
                }
            }
        }
        
        return minAmbiguity;
    }

    public void setMegaTag2(boolean value) {
        this.useMegaTag2 = value;
        limelightIOs.forEach((key, io) -> {
            io.setMegaTag2(useMegaTag2);
        });
    }

    public Optional<PoseEstimate[]> determinePoseEstimate(AngularVelocity gyroRate) {
        limelightIOs.forEach((key, io) -> {
            LimelightIOInputsAutoLogged input = limelightInputs.get(key);
            if (input != null) {
                io.setNewEstimate(input, !rejectUpdate(input.poseBlue, gyroRate));
            }
        });

        LimelightIOInputsAutoLogged rightInputs = limelightInputs.get(LIMELIGHT_RIGHT);
        LimelightIOInputsAutoLogged leftInputs = limelightInputs.get(LIMELIGHT_LEFT);

        if (rightInputs == null || leftInputs == null) {
            return Optional.empty();
        }

        boolean newRightEstimate = rightInputs.newEstimate;
        boolean newLeftEstimate = leftInputs.newEstimate;
        PoseEstimate lastEstimateRight = rightInputs.poseBlue;
        PoseEstimate lastEstimateLeft = leftInputs.poseBlue;
        // No valid pose estimates :(
        if (!newRightEstimate && !newLeftEstimate) {
            return Optional.empty();

        } else if (newRightEstimate && !newLeftEstimate) {
            // One valid pose estimate (right)
            limelightIOs.get(LIMELIGHT_RIGHT).setNewEstimate(rightInputs, false);
            return Optional.of(new PoseEstimate[]{lastEstimateRight, null});

        } else if (!newRightEstimate) {
            // One valid pose estimate (left)
            limelightIOs.get(LIMELIGHT_LEFT).setNewEstimate(leftInputs, false);
            return Optional.of(new PoseEstimate[]{lastEstimateLeft, null});

        } else {
            // Two valid pose estimates, disgard the one that's further
            limelightIOs.get(LIMELIGHT_RIGHT).setNewEstimate(rightInputs, false);
            limelightIOs.get(LIMELIGHT_LEFT).setNewEstimate(leftInputs, false);
            return Optional.of(new PoseEstimate[]{lastEstimateRight, lastEstimateLeft});
        }
    }

    private void addVisionMeasurement(Optional<PoseEstimate[]> estimatedPose) {
        if (estimatedPose.isPresent()) {
            if (estimatedPose.get()[0] != null) {
                if (useMegaTag2) {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[0].timestampSeconds(), new Pose2d(estimatedPose.get()[0].pose().getTranslation(), estimatedPose.get()[0].pose().getRotation().minus(Rotation2d.fromDegrees(180))), VecBuilder.fill(.7, .7, 9999999));
                } else {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[0].timestampSeconds(), new Pose2d(estimatedPose.get()[0].pose().getTranslation(), estimatedPose.get()[0].pose().getRotation().minus(Rotation2d.fromDegrees(180))), VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput(LIMELIGHT_LEFT + "/estimatedPose", estimatedPose.get()[0]);
            }
            if (estimatedPose.get()[1] != null) {
                if (useMegaTag2) {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[1].timestampSeconds(), new Pose2d(estimatedPose.get()[1].pose().getTranslation(), estimatedPose.get()[1].pose().getRotation().minus(Rotation2d.fromDegrees(180))), VecBuilder.fill(.7, .7, 9999999));
                } else {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[1].timestampSeconds(), new Pose2d(estimatedPose.get()[1].pose().getTranslation(), estimatedPose.get()[1].pose().getRotation().minus(Rotation2d.fromDegrees(180))), VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput(LIMELIGHT_RIGHT + "/estimatedPose",estimatedPose.get()[1]);
            }
        }
    }


    @Override
    public void periodic() {
        limelightIOs.forEach((name, io) -> {
            // TODO: figure out why 180 - abs()
            io.setRobotOrientation(
                    AllianceFlipUtil.shouldFlip() ?
                            (-1 * AllianceFlipUtil.apply(swerveLocalizer.getLatestPose().getRotation()).getDegrees()) :
                            ((180 - Math.abs(swerveLocalizer.getLatestPose().getRotation().getDegrees()))
                                    * (swerveLocalizer.getLatestPose().getRotation().getDegrees() > 0 ? -1 : 1))
                    , 0, 0, 0, 0, 0);
        });

        AngularVelocity gyroRate = Units.DegreesPerSecond.of(swerveLocalizer.getSmoothedVelocity().getRotation().getDegrees());

        limelightIOs.forEach((name, io) -> {
            LimelightIOInputsAutoLogged input = limelightInputs.get(name);
            if (input != null) {
                io.updateInputs(input, gyroRate);
                Logger.processInputs(name, input);
            }
        });
        estimatedPose = determinePoseEstimate(gyroRate);
        addVisionMeasurement(estimatedPose);
        LoggedTracer.record("Limelight");
    }
}

