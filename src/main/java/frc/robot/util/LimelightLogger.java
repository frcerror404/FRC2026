package frc.robot.util;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightData;
import limelight.networktables.LimelightPipelineData;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightTargetData;
import limelight.networktables.PoseEstimate;
import limelight.results.HardwareReport;
import limelight.results.IMUResults;
import limelight.results.RawFiducial;
import limelight.results.RewindStats;
import org.littletonrobotics.junction.Logger;

public final class LimelightLogger {
  private LimelightLogger() {}

  public static void log(Limelight limelight) {
    log(limelight, null);
  }

  public static void log(Limelight limelight, Optional<PoseEstimate> poseEstimate) {
    if (limelight == null) {
      return;
    }

    String base = "Limelight/" + limelight.limelightName;
    LimelightData data = limelight.getData();
    LimelightTargetData targetData = data.targetData;
    LimelightPipelineData pipelineData = data.pipelineData;

    Logger.recordOutput(base + "/Target/Valid", targetData.getTargetStatus());
    Logger.recordOutput(base + "/Target/Count", targetData.getTargetCount());
    Logger.recordOutput(base + "/Target/AprilTagId", targetData.getAprilTagID());
    Logger.recordOutput(base + "/Target/TxDegrees", targetData.getHorizontalOffset());
    Logger.recordOutput(base + "/Target/TyDegrees", targetData.getVerticalOffset());
    Logger.recordOutput(base + "/Target/Area", targetData.getTargetArea());

    Logger.recordOutput(base + "/Pipeline/Index", pipelineData.getCurrentPipelineIndex());
    Logger.recordOutput(base + "/Pipeline/Type", pipelineData.getCurrentPipelineType());
    Logger.recordOutput(base + "/Pipeline/LatencyMs", pipelineData.getProcessingLatency());
    Logger.recordOutput(base + "/Pipeline/CaptureLatencyMs", pipelineData.getCaptureLatency());

    logRawFiducials(base + "/RawFiducials", data.getRawFiducials());

    if (poseEstimate != null) {
      logPoseEstimate(base + "/PoseEstimate", poseEstimate);
    }

    limelight.getLatestResults().ifPresent((results) -> logResults(base + "/Status", results));
  }

  private static void logPoseEstimate(String base, Optional<PoseEstimate> poseEstimate) {
    boolean hasData = poseEstimate != null && poseEstimate.isPresent();
    Logger.recordOutput(base + "/HasData", hasData);
    if (!hasData) {
      Logger.recordOutput(base + "/TagCount", 0);
      Logger.recordOutput(base + "/AvgTagDistMeters", 0.0);
      Logger.recordOutput(base + "/AvgTagArea", 0.0);
      Logger.recordOutput(base + "/MinTagAmbiguity", 1.0);
      Logger.recordOutput(base + "/Pose", new Pose2d());
      logRawFiducials(base + "/RawFiducials", new RawFiducial[0]);
      return;
    }

    PoseEstimate estimate = poseEstimate.get();
    Logger.recordOutput(base + "/TagCount", estimate.tagCount);
    Logger.recordOutput(base + "/AvgTagDistMeters", estimate.avgTagDist);
    Logger.recordOutput(base + "/AvgTagArea", estimate.avgTagArea);
    Logger.recordOutput(base + "/MinTagAmbiguity", estimate.getMinTagAmbiguity());
    Logger.recordOutput(base + "/Pose", estimate.pose.toPose2d());
    logRawFiducials(base + "/RawFiducials", estimate.rawFiducials);
  }

  private static void logRawFiducials(String base, RawFiducial[] fiducials) {
    int count = fiducials == null ? 0 : fiducials.length;
    double[] ids = new double[count];
    double[] txnc = new double[count];
    double[] tync = new double[count];
    double[] area = new double[count];
    double[] distToCamera = new double[count];
    double[] distToRobot = new double[count];
    double[] ambiguity = new double[count];

    for (int i = 0; i < count; i++) {
      RawFiducial fiducial = fiducials[i];
      ids[i] = fiducial.id;
      txnc[i] = fiducial.txnc;
      tync[i] = fiducial.tync;
      area[i] = fiducial.ta;
      distToCamera[i] = fiducial.distToCamera;
      distToRobot[i] = fiducial.distToRobot;
      ambiguity[i] = fiducial.ambiguity;
    }

    Logger.recordOutput(base + "/Count", count);
    Logger.recordOutput(base + "/Ids", ids);
    Logger.recordOutput(base + "/Txnc", txnc);
    Logger.recordOutput(base + "/Tync", tync);
    Logger.recordOutput(base + "/Area", area);
    Logger.recordOutput(base + "/DistToCameraMeters", distToCamera);
    Logger.recordOutput(base + "/DistToRobotMeters", distToRobot);
    Logger.recordOutput(base + "/Ambiguity", ambiguity);
  }

  private static void logResults(String base, LimelightResults results) {
    Logger.recordOutput(
        base + "/HardwareType", results.hardwareType == null ? "" : results.hardwareType);
    Logger.recordOutput(
        base + "/ImageSource", results.imageSource == null ? "" : results.imageSource);

    HardwareReport hardware = results.hardware;
    if (hardware != null) {
      Logger.recordOutput(base + "/Hardware/CameraId", hardware.cameraId);
      Logger.recordOutput(base + "/Hardware/CpuPercent", hardware.cpuUsage);
      Logger.recordOutput(base + "/Hardware/RamPercent", hardware.ramUsage);
      Logger.recordOutput(base + "/Hardware/DiskFreeMb", hardware.diskFree);
      Logger.recordOutput(base + "/Hardware/DiskTotalMb", hardware.diskTotal);
      Logger.recordOutput(base + "/Hardware/TempC", hardware.getTemp().in(Celsius));
    }

    IMUResults imuResults = results.imuResults;
    if (imuResults != null) {
      Logger.recordOutput(base + "/IMU/FusedYawDeg", imuResults.getFusedYaw().in(Degrees));
      if (imuResults.data.length >= 6) {
        Logger.recordOutput(base + "/IMU/RawYawDeg", imuResults.data[2]);
        Logger.recordOutput(base + "/IMU/RawYawRateDegPerSec", imuResults.data[5]);
      }
    }

    RewindStats rewindStats = results.rewindStats;
    if (rewindStats != null) {
      Logger.recordOutput(base + "/Rewind/Enabled", rewindStats.getEnabled());
      Logger.recordOutput(base + "/Rewind/BufferUsage", rewindStats.bufferUsage);
      Logger.recordOutput(base + "/Rewind/StoredSeconds", rewindStats.getBufferTime().in(Seconds));
      Logger.recordOutput(base + "/Rewind/FrameCount", rewindStats.getFrameCount());
      Logger.recordOutput(
          base + "/Rewind/LatencyPenaltyUs", rewindStats.getLatencyPenalty().in(Microseconds));
      Logger.recordOutput(base + "/Rewind/Flushing", rewindStats.isFlushing());
    }
  }
}
