package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import lib.ironpulse.math.obstacle.Obstacle2d;
import lib.ironpulse.math.obstacle.PolygonObstacle2d;
import lib.ntext.NTParameter;
import lombok.AllArgsConstructor;
import lombok.Data;

import javax.swing.text.html.Option;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

public class CoralRecorder {
  public static Obstacle2d kRedReefHexagon = new PolygonObstacle2d(
      new Translation2d(14.50, 4.858),
      new Translation2d(13.059, 5.689),
      new Translation2d(11.618, 4.858),
      new Translation2d(11.618, 3.194),
      new Translation2d(13.059, 2.362),
      new Translation2d(14.500, 3.194)
  );
  public static Obstacle2d kBlueReefHexagon = new PolygonObstacle2d(
      new Translation2d(FieldConstants.fieldLength - 14.50, 4.858),
      new Translation2d(FieldConstants.fieldLength - 13.059, 5.689),
      new Translation2d(FieldConstants.fieldLength - 11.618, 4.858),
      new Translation2d(FieldConstants.fieldLength - 11.618, 3.194),
      new Translation2d(FieldConstants.fieldLength - 13.059, 2.362),
      new Translation2d(FieldConstants.fieldLength - 14.500, 3.194)
  );

  public int currentId = 0;
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
    if (kRedReefHexagon.isInside(loc)) return;
    if (kBlueReefHexagon.isInside(loc)) return;

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
      nearest.setConfidence(
          Math.min(1.0, nearest.getConfidence() + dt * CoralRecorderParamsNT.confidenceTimeObservationGain.getValue())
      );
      nearest.setTranslation(
          nearest.getTranslation().interpolate(loc, CoralRecorderParamsNT.confidenceNewObservationProportion.getValue())
      );
    } else {
      // does not have any near coral, create a new one
      CoralInfo info = new CoralInfo(
          currentId++, loc, 0.0, CoralRecorderParamsNT.confidenceStart.getValue(), true
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

  public Optional<CoralInfo> getCoralById(int id) {
    for(CoralInfo info : coralInfos) {
      if (info.getId() == id) return Optional.of(info);
    }
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

  public Optional<CoralInfo> getNearestCoralInSight(Pose2d robotPose, Angle insightAngle) {
    var pRobot = robotPose.getTranslation();
    var heading = robotPose.getRotation();

    CoralInfo nearest = null;
    double minDist = Double.MAX_VALUE;

    for (CoralInfo info : coralInfos) {
      if (info.getConfidence() <= CoralRecorderParamsNT.confidenceThreshold.getValue()) {
        continue;
      }

      // vector from robot to this coral
      Translation2d toCoral = info.getTranslation().minus(pRobot);
      Rotation2d toCoralAngle = toAngle(toCoral);
      double dist = toCoral.getNorm();

      // within field of view and closer than any before
      if (epsilonEquals(heading, toCoralAngle, insightAngle.in(Radians)) && dist < minDist) {
        minDist = dist;
        nearest = info;
      }
    }

    return nearest != null ? Optional.of(nearest) : Optional.empty();
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
    public int id;

    public Translation2d translation;
    public double addedTime;
    public double confidence;
    public boolean hasUpdate;
  }

  @NTParameter(tableName = "Params/CoralRecorder")
  public static class CoralRecorderParams {
    static final double sameCoralRadiusMeters = 0.20;
    static final double confidenceStart = 0.45;
    static final double confidenceTimeDecay = 0.3; // decay in confidence per second
    static final double confidenceTimeObservationGain = 8.0; // increase in confidence per second when observed
    static final double confidenceNewObservationProportion = 0.9; // starting confidence for a new observation
    static final double confidenceThreshold = 0.6;
  }
}
