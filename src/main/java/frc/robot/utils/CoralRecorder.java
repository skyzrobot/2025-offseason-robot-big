package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import lib.ntext.NTParameter;
import lombok.Data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

public class CoralRecorder {
  @Data
  public static class CoralInfo {
    public Translation2d translation;
    public double addedTime;
    public double confidence;
  }

  public List<CoralInfo> coralInfos = new ArrayList<>();

  private static CoralRecorder instance;

  private CoralRecorder() {}

  public static CoralRecorder getInstance() {
    if (instance == null) {
      instance = new CoralRecorder();
    }
    return instance;
  }

  public void update(double dt) {
    Iterator<CoralInfo> iterator = coralInfos.iterator();
    while (iterator.hasNext()) {
      CoralInfo info = iterator.next();
      info.setAddedTime(info.getAddedTime() + dt);
      info.setConfidence(info.getConfidence() - dt / CoralRecorderParams.confidenceTimeDecayGain);
      if (info.getConfidence() <= 0.0) {
        iterator.remove();
      }
    }
  }

  public void addCoralMeasurement(Pose3d pose) {
    Translation2d newTrans = new Translation2d(pose.getX(), pose.getY());
    CoralInfo nearest = null;
    double minDistance = Double.MAX_VALUE;
    for (CoralInfo info : coralInfos) {
      double distance = info.getTranslation().getDistance(newTrans);
      if (distance < minDistance) {
        minDistance = distance;
        nearest = info;
      }
    }
    if (nearest != null && minDistance <= CoralRecorderParams.sameCoralRadiusMeters) {
      nearest.setConfidence(Math.min(1.0, nearest.getConfidence()
          + CoralRecorderParams.confidenceTimeObservationGain));
      double avgX = (nearest.getTranslation().getX() + newTrans.getX()) / 2.0;
      double avgY = (nearest.getTranslation().getY() + newTrans.getY()) / 2.0;
      nearest.setTranslation(new Translation2d(avgX, avgY));
    } else {
      CoralInfo info = new CoralInfo();
      info.setTranslation(newTrans);
      info.setAddedTime(0.0);
      info.setConfidence(CoralRecorderParams.confidenceStart);
      coralInfos.add(info);
    }
  }

  public Optional<CoralInfo> getBestCoral(Pose2d poseWorldRobot) {

    if (coralInfos.isEmpty()) {
      return Optional.empty();
    }
    Translation2d robotPos = poseWorldRobot.getTranslation();
    CoralInfo nearest = null;
    double minDistance = Double.MAX_VALUE;
    for (CoralInfo info : coralInfos) {
      double distance = info.getTranslation().getDistance(robotPos);
      if (distance < minDistance) {
        minDistance = distance;
        nearest = info;
      }
    }
    return Optional.ofNullable(nearest);
  }

  @NTParameter(tableName = "Params/CoralRecorder")
  public static class CoralRecorderParams {
    static final double sameCoralRadiusMeters = 0.2;
    static final double confidenceStart = 0.3;
    static final double confidenceTimeDecayGain = 10.0;
    static final double confidenceTimeObservationGain = 0.2;
  }
}
