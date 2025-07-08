package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import lib.ntext.NTParameter;
import lombok.AllArgsConstructor;
import lombok.Data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class CoralRecorder {
  public List<CoralInfo> coralInfos = new ArrayList<>();

  public void update(double dt) {
    Iterator<CoralInfo> iterator = coralInfos.iterator();
    while (iterator.hasNext()) {
      CoralInfo info = iterator.next();

      // increase time
      info.setAddedTime(info.getAddedTime() + dt);
      // decay confidence
      info.setConfidence(info.getConfidence() - dt / CoralRecorderParamsNT.confidenceTimeDecay.getValue());
      if (info.getConfidence() <= 0.0) {
        iterator.remove();
      }
    }
  }

  public void addCoralMeasurement(Translation2d loc, double dt) {
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
          Math.min(1.0, nearest.getConfidence() + dt / CoralRecorderParamsNT.confidenceTimeObservationGain.getValue())
      );
      nearest.setTranslation(
          nearest.getTranslation().interpolate(loc, CoralRecorderParamsNT.confidenceNewObservationProportion.getValue())
      );
    } else {
      // does not have any near coral, create a new one
      CoralInfo info = new CoralInfo(
          loc, 0.0, CoralRecorderParamsNT.confidenceStart.getValue()
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

  public Translation2d[] getCoralLocations() {
    double threshold = CoralRecorderParamsNT.confidenceThreshold.getValue();
    return coralInfos.stream()
        // only include reliable corals
        .filter(info -> info.getConfidence() > threshold)
        // extract their positions
        .map(CoralInfo::getTranslation)
        // collect into a read-only List
        .toList().toArray(new Translation2d[0]);
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
  }

  @NTParameter(tableName = "Params/CoralRecorder")
  public static class CoralRecorderParams {
    static final double sameCoralRadiusMeters = 0.25;
    static final double confidenceStart = 0.3;
    static final double confidenceTimeDecay = 0.7; // decay in confidence per second
    static final double confidenceTimeObservationGain = 2.0; // increase in confidence per second when observed
    static final double confidenceNewObservationProportion = 0.3; // starting confidence for a new observation
    static final double confidenceThreshold = 0.6;
  }
}
