package frc.robot.subsystems.photonvision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.PhotonVisionParamsNT;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_ENABLED;
import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private Timer snapshotTimer = new Timer();

    public PhotonVisionSubsystem(PhotonVisionIO... ios) {
        this.ios = ios;
        inputs = new PhotonVisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PhotonVisionIOInputsAutoLogged();
        }
        snapshotTimer.start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("PhotonVision/Inst" + i, inputs[i]);
        }
        if (snapshotTimer.hasElapsed(SNAPSHOT_PERIOD)) {
            for (int i = 0; i < ios.length; i++) {
                if (SNAPSHOT_ENABLED[i]) ios[i].takeOutputSnapshot();
            }
            snapshotTimer.reset();
        }
        
        // Perform 3D projection for all detected targets
        performPeriodicProjections();
    }
    
    /**
     * Projects fresh detections to world coordinates and logs the results
     * Only processes new vision data to prevent objects from moving with the robot
     */
    private void performPeriodicProjections() {
        // Process only fresh detections to prevent stale coordinate transformations
        List<RawDetection> freshDetections = getFreshRawDetections();
        
        for (RawDetection detection : freshDetections) {
            // Project to robot-relative 3D pose
            Pose3d robotRelativePose = projectTargetTo3D(detection);
            
            // Get robot pose from when detection was captured (accounts for robot movement during processing)
            double detectionTime = detection.timestampMs / 1000.0;
            Pose3d robotPoseAtDetection = RobotStateRecorder.getInstance().getTransform(
                Seconds.of(detectionTime),
                lib.ironpulse.rbd.TransformRecorder.kFrameWorld,
                lib.ironpulse.rbd.TransformRecorder.kFrameRobot
            ).orElse(RobotStateRecorder.getPoseWorldRobotCurrent());
            
            // Transform to world coordinates
            Pose3d worldPose = robotPoseAtDetection.transformBy(
                new Transform3d(robotRelativePose.getTranslation(), robotRelativePose.getRotation()));

            // Add to RobotStateRecorder
            RobotStateRecorder.addCoralMeasurement(worldPose.toPose2d().getTranslation());
            
            // Log poses
            String basePath = "PhotonVision/Camera" + detection.cameraId + "/Target" + getTargetIndex(detection);
            Logger.recordOutput(basePath + "/RobotRelativePose3D", robotRelativePose);
            Logger.recordOutput(basePath + "/WorldPose3D", worldPose);
        }
        
        // Log detection counts
        Logger.recordOutput("PhotonVision/TotalDetections", getAllRawDetections().size());
        Logger.recordOutput("PhotonVision/FreshDetections", freshDetections.size());
    }
    
    /**
     * Gets the target index within a camera for logging purposes
     * @param detection The detection to find the index for
     * @return The target index within its camera
     */
    private int getTargetIndex(RawDetection detection) {
        List<RawDetection> cameraDetections = getDetectionsFromCamera(detection.cameraId);
        for (int i = 0; i < cameraDetections.size(); i++) {
            if (cameraDetections.get(i).equals(detection)) {
                return i;
            }
        }
        return 0; // Fallback
    }



    /**
     * Gets all raw detection data from all cameras
     * @return List of all raw detections with camera information
     */
    public List<RawDetection> getAllRawDetections() {
        List<RawDetection> allDetections = new ArrayList<>();
        
        for (int i = 0; i < inputs.length; i++) {
            PhotonVisionIOInputsAutoLogged input = inputs[i];
            if (input.hasTargets && input.targetCount > 0) {
                for (int j = 0; j < input.targetCount; j++) {
                    double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                    double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                    double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                    double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                    double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                    int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                    double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                    double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                    
                    allDetections.add(new RawDetection(
                        i, // camera ID
                        input.timestampMs,
                        yaw,
                        pitch,
                        area,
                        skew,
                        ambiguity,
                        fiducialId,
                        pixelX,
                        pixelY
                    ));
                }
            }
        }
        
        return allDetections;
    }

    /**
     * Gets only fresh raw detection data from all cameras (for coordinate transformations)
     * @return List of fresh raw detections with camera information
     */
    public List<RawDetection> getFreshRawDetections() {
        List<RawDetection> freshDetections = new ArrayList<>();
        
        for (int i = 0; i < inputs.length; i++) {
            PhotonVisionIOInputsAutoLogged input = inputs[i];
            // Only process detections when we have fresh data
            if (input.hasFreshData && input.hasTargets && input.targetCount > 0) {
                for (int j = 0; j < input.targetCount; j++) {
                    double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                    double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                    double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                    double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                    double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                    int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                    double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                    double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                    
                    freshDetections.add(new RawDetection(
                        i, // camera ID
                        input.timestampMs,
                        yaw,
                        pitch,
                        area,
                        skew,
                        ambiguity,
                        fiducialId,
                        pixelX,
                        pixelY
                    ));
                }
            }
        }
        
        return freshDetections;
    }

    /**
     * Gets raw detection data from a specific camera
     * @param cameraId The ID of the camera
     * @return List of raw detections from the specified camera
     */
    public List<RawDetection> getDetectionsFromCamera(int cameraId) {
        if (cameraId < 0 || cameraId >= inputs.length) {
            return List.of();
        }
        
        List<RawDetection> detections = new ArrayList<>();
        PhotonVisionIOInputsAutoLogged input = inputs[cameraId];
        
        if (input.hasTargets && input.targetCount > 0) {
            for (int j = 0; j < input.targetCount; j++) {
                double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;
                
                detections.add(new RawDetection(
                    cameraId,
                    input.timestampMs,
                    yaw,
                    pitch,
                    area,
                    skew,
                    ambiguity,
                    fiducialId,
                    pixelX,
                    pixelY
                ));
            }
        }
        
        return detections;
    }

    /**
     * Gets the largest target (by area) from all cameras
     * @return The largest target detection, or null if no targets
     */
    public RawDetection getLargestTarget() {
        RawDetection largest = null;
        double maxArea = 0.0;
        
        for (RawDetection detection : getAllRawDetections()) {
            if (detection.area > maxArea) {
                maxArea = detection.area;
                largest = detection;
            }
        }
        
        return largest;
    }

    /**
     * Gets the target closest to the crosshair (smallest yaw angle) from all cameras
     * @return The target closest to crosshair, or null if no targets
     */
    public RawDetection getClosestToCrosshair() {
        RawDetection closest = null;
        double minYaw = Double.MAX_VALUE;
        
        for (RawDetection detection : getAllRawDetections()) {
            double absYaw = Math.abs(detection.yaw);
            if (absYaw < minYaw) {
                minYaw = absYaw;
                closest = detection;
            }
        }
        
        return closest;
    }

    /**
     * Checks if any camera has targets
     * @return true if any camera has targets, false otherwise
     */
    public boolean hasAnyTargets() {
        for (PhotonVisionIOInputsAutoLogged input : inputs) {
            if (input.hasTargets) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the total number of targets detected across all cameras
     * @return Total number of targets
     */
    public int getTotalTargetCount() {
        int total = 0;
        for (PhotonVisionIOInputsAutoLogged input : inputs) {
            total += input.targetCount;
        }
        return total;
    }

    /**
     * Projects a 2D detection to 3D pose relative to the robot center
     * @param detection The raw detection data
     * @return Estimated 3D pose of the target relative to the robot center
     */
    public Pose3d projectTargetTo3D(RawDetection detection) {
        // Get camera configuration
        double cameraHeight = PhotonVisionParamsNT.CAMERA_HEIGHT_METERS.getValue();
        double cameraPitch = PhotonVisionParamsNT.CAMERA_PITCH_DEGREES.getValue();
        double groundHeight = PhotonVisionParamsNT.GROUND_HEIGHT_METERS.getValue();
        double distanceScale = PhotonVisionParamsNT.DISTANCE_SCALE_FACTOR.getValue();
        
        // Convert to radians
        double yawRad = Units.degreesToRadians(detection.yaw);
        double pitchRad = Units.degreesToRadians(detection.pitch);
        double cameraPitchRad = Units.degreesToRadians(cameraPitch);
        
        // Calculate distance using trigonometry
        double heightDiff = cameraHeight - groundHeight;
        double totalPitch = pitchRad + cameraPitchRad;
        double distance = Math.abs(totalPitch) > 0.01 ? 
            Math.abs(heightDiff / Math.tan(totalPitch)) * distanceScale : 1.0;
        
        // Calculate 3D position (correct for PhotonVision coordinate system)
        double x = distance * Math.cos(yawRad);     // Forward/backward
        double y = -distance * Math.sin(yawRad);    // Left/right (inverted for robot coords)
        double z = -(cameraHeight - groundHeight);  // Height relative to camera
        
        // Create and transform pose
        Pose3d cameraRelativePose = new Pose3d(x, y, z, new Rotation3d());
        Pose3d robotRelativePose = transformCameraToRobot(cameraRelativePose);
        
        // Log essential data
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/Distance", distance);
        Logger.recordOutput("PhotonVision/Camera" + detection.cameraId + "/RobotRelativePose3D", robotRelativePose);
        
        return robotRelativePose;
    }
    
    /**
     * Transforms a pose from camera coordinates to robot coordinates
     * @param cameraRelativePose The pose relative to the camera
     * @return The pose relative to the robot center
     */
    private Pose3d transformCameraToRobot(Pose3d cameraRelativePose) {
        // Get camera position relative to robot center from constants
        double cameraToRobotX = PhotonVisionParamsNT.CAMERA_TO_ROBOT_X.getValue();
        double cameraToRobotY = PhotonVisionParamsNT.CAMERA_TO_ROBOT_Y.getValue();
        double cameraToRobotZ = PhotonVisionParamsNT.CAMERA_TO_ROBOT_Z.getValue();
        double cameraToRobotRotationDegrees = PhotonVisionParamsNT.CAMERA_TO_ROBOT_ROTATION_DEGREES.getValue();
        
        // Create the camera-to-robot transform
        Translation3d cameraToRobotTranslation = new Translation3d(cameraToRobotX, cameraToRobotY, cameraToRobotZ);
        Rotation3d cameraToRobotRotation = new Rotation3d(0, 0, Units.degreesToRadians(cameraToRobotRotationDegrees));
        Transform3d cameraToRobotTransform = new Transform3d(cameraToRobotTranslation, cameraToRobotRotation);
        
        // Apply the transform to get robot-relative pose
        return cameraRelativePose.transformBy(cameraToRobotTransform);
    }
    
    /**
     * Projects all detected targets to 3D poses relative to the robot center
     * @return List of 3D poses for all detected targets relative to robot center
     */
    public List<Pose3d> projectAllTargetsTo3D() {
        List<Pose3d> poses = new ArrayList<>();
        
        for (RawDetection detection : getAllRawDetections()) {
            poses.add(projectTargetTo3D(detection));
        }
        
        return poses;
    }



    /**
     * Record class to hold raw detection data
     */
    public record RawDetection(
        int cameraId,
        long timestampMs,
        double yaw,         // degrees, positive = right
        double pitch,       // degrees, positive = up
        double area,        // percent (0-100)
        double skew,        // degrees
        double ambiguity,   // 0-1, lower is better
        int fiducialId,     // -1 for objects, positive for AprilTags
        double pixelX,      // pixel X coordinate of target center
        double pixelY       // pixel Y coordinate of target center
    ) {
        
        public boolean isAprilTag() {
            return fiducialId >= 0;
        }
        
        public boolean isObject() {
            return fiducialId < 0;
        }
        
        public double getConfidence() {
            return Math.max(0.0, 1.0 - ambiguity);
        }
    }
}
