package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import lib.ntext.NTParameter;
import lombok.AllArgsConstructor;
import lombok.Data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class CoralRecorder {
  public List<CoralInfo> coralInfos = new ArrayList<>();

  public void update(double dt) {
    Iterator<CoralInfo> iterator = coralInfos.iterator();
    while (iterator.hasNext()) {
      CoralInfo info = iterator.next();
      // increase time
      info.setAddedTime(info.getAddedTime() + dt);
      // decay confidence
      info.setConfidence(info.getConfidence() - dt * CoralRecorderParamsNT.confidenceTimeDecay.getValue());
      if (info.getConfidence() <= 0.0) iterator.remove();
    }
  }

  public void addCoralMeasurement(Translation2d loc, double dt) {
    if (loc.getX() < 0.0 || loc.getX() > FieldConstants.fieldLength) return;
    if (loc.getY() < 0.0 || loc.getY() > FieldConstants.fieldWidth) return;

    // find nearest coral
    CoralInfo nearest = null;
    double minDistance = Double.MAX_VALUE;
    for (CoralInfo info : coralInfos) {
      double distance = info.getTranslation().getDistance(loc);
      if (distance < minDistance) {
        minDistance = distance;
        nearest = info;
      }
    }

    // if have near coral and within radius, update to current
    if (nearest != null && minDistance <= CoralRecorderParamsNT.sameCoralRadiusMeters.getValue()) {
      System.out.println("Before: "  + nearest.getConfidence());
      nearest.setConfidence(
          Math.min(1.0, nearest.getConfidence() + dt *  CoralRecorderParamsNT.confidenceTimeObservationGain.getValue())
      );
      System.out.println("After: "  + nearest.getConfidence());
      nearest.setTranslation(
          nearest.getTranslation().interpolate(loc, CoralRecorderParamsNT.confidenceNewObservationProportion.getValue())
      );
    } else {
      // does not have any near coral, create a new one
      CoralInfo info = new CoralInfo(
          loc, 0.0, CoralRecorderParamsNT.confidenceStart.getValue(), true
      );
      coralInfos.add(info);
    }
  }

  public Optional<CoralInfo> getNearestCoral(Pose2d robotPose) {
    var pRobot = robotPose.getTranslation();
    CoralInfo nearest = null;
    double minDistance = Double.MAX_VALUE;
    for (CoralInfo info : coralInfos) {
      if (info.getConfidence() <= CoralRecorderParamsNT.confidenceThreshold.getValue()) continue;
      double distance = info.getTranslation().getDistance(pRobot);
      if (distance < minDistance) {
        minDistance = distance;
        nearest = info;
      }
    }

    if (nearest != null) return Optional.of(nearest);
    return Optional.empty();
  }

  public Optional<CoralInfo> getMostInDirectionCoral(Pose2d robotPose) {
    var pRobot = robotPose.getTranslation();
    var robotHeading = robotPose.getRotation();
    
    CoralInfo mostAligned = null;
    double maxDotProduct = Double.NEGATIVE_INFINITY;
    
    for (CoralInfo info : coralInfos) {
      if (info.getConfidence() <= CoralRecorderParamsNT.confidenceThreshold.getValue()) continue;
      
      // Calculate vector from robot to coral
      Translation2d robotToCoral = info.getTranslation().minus(pRobot);
      
      // Skip if coral is at robot position
      if (robotToCoral.getNorm() == 0.0) continue;
      
      // Normalize the vector to get direction
      Translation2d directionToCoral = robotToCoral.div(robotToCoral.getNorm());
      
      // Calculate dot product with robot's heading direction
      Translation2d robotHeadingVector = new Translation2d(robotHeading.getCos(), robotHeading.getSin());
      double dotProduct = directionToCoral.getX() * robotHeadingVector.getX() + 
                         directionToCoral.getY() * robotHeadingVector.getY();
      
      // Find coral with highest dot product (most aligned with robot heading)
      if (dotProduct > maxDotProduct) {
        maxDotProduct = dotProduct;
        mostAligned = info;
      }
    }
    
    if (mostAligned != null) return Optional.of(mostAligned);
    return Optional.empty();
  }

  public Pose2d[] getCoralLocations() {
    double threshold = CoralRecorderParamsNT.confidenceThreshold.getValue();

    return coralInfos.stream()
        // only include reliable corals
        .filter(info -> info.getConfidence() > threshold)
        // extract their positions
        .map(info -> new Pose2d(info.getTranslation(), Rotation2d.kZero))
        // collect into a read-only List
        .toList().toArray(new Pose2d[0]);
  }

  public void reset() {
    coralInfos.clear();
  }

  @Data
  @AllArgsConstructor
  public static class CoralInfo {
    public Translation2d translation;
    public double addedTime;
    public double confidence;
    public boolean hasUpdate;
  }

  @NTParameter(tableName = "Params/CoralRecorder")
  public static class CoralRecorderParams {
    static final double sameCoralRadiusMeters = 0.35;
    static final double confidenceStart = 0.3;
    static final double confidenceTimeDecay = 0.7; // decay in confidence per second
    static final double confidenceTimeObservationGain = 5.0; // increase in confidence per second when observed
    static final double confidenceNewObservationProportion = 0.2; // starting confidence for a new observation
    static final double confidenceThreshold = 0.5;
  }
}
