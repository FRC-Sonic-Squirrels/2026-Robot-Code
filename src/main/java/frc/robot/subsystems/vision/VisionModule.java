package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team6328.Alert;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.Inputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class VisionModule {
  private final VisionIO            visionIO;
  private final Inputs              visionIOInputs;
  private final PhotonPoseEstimator photonPoseEstimator;
  final Transform3d                 RobotToCamera;
  final String                      name;
  final int                         id;

  public VisionResultLoggedFields loggedFields;
  public double                   lastSuccessfullyProcessedResultTimeStampCTRETime;
  public VisionResultStatus       lastStatus;

  public final Alert missingCameraAlert;

  // Logging

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.ByteArray photonPacketBytes;
    public LoggerEntry.Decimal   lastTimestampCTRETime;
    public LoggerEntry.Bool      connected;
    public LoggerEntry.Decimal   medianLatency;
    public LoggerEntry.Decimal   medianUpdateTime;

    public LoggerEntry.Decimal        lastTimestampCTRETimeSuccessful;
    public LoggerEntry.Text           status;
    public LoggerEntry.Struct<Pose3d> cameraPose;
    public LoggerEntry.Struct<Pose3d> calculatedRobotPose3d;
    public LoggerEntry.Struct<Pose2d> calculatedRobotPose2d;
    public LoggerEntry.Integer        numSeenTargets;
    public LoggerEntry.Decimal        distanceFromExistingPoseEstimate;
    public LoggerEntry.Decimal        averageDistanceFromTags;
    public LoggerEntry.Decimal        tagAmbiguity;
    public LoggerEntry.Decimal        xyStandardDeviation;
    public LoggerEntry.Decimal        thetaStandardDeviation;
    public LoggerEntry.Bool           successfulResult;
    public LoggerEntry.DecimalArray   seenTargetsIDs;

    public LoggerEntry.Struct<Pose3d> distanceToTag;
    public LoggerEntry.Decimal        yawToTag;
  }

  private final int TAG_ID = 15;

  private final Logs logs;

  private final AprilTagFieldLayout aprilTagLayout;

  public VisionModule(LoggerGroup logGroup, VisionModuleConfiguration config, AprilTagFieldLayout aprilTagFieldLayout,
      int id) {
    this.visionIO       = config.visionIO;
    this.visionIOInputs = new Inputs();
    this.RobotToCamera  = config.robotToCamera;
    this.name           = config.logName;
    this.id             = id;

    this.loggedFields = VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INIT);

    this.lastSuccessfullyProcessedResultTimeStampCTRETime = -1.0;

    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, RobotToCamera);

    this.missingCameraAlert = new Alert("Missing Camera: " + this.name, Alert.AlertType.ERROR);

    this.aprilTagLayout = aprilTagFieldLayout;

    // Logging

    logs = logGroup.newAutoLogger(Logs.class, name);
  }

  public void log(Pose3d robotPose, double currentTime) {
    if (Constants.unusedCode) {
      var    pipelineResult    = visionIOInputs.lastResult.result();
      byte[] photonPacketBytes = new byte[pipelineResult.getPacketSize()];
      pipelineResult.getSerde().pack(new Packet(photonPacketBytes), pipelineResult);
      logs.photonPacketBytes.info(photonPacketBytes);
    }
    logs.lastTimestampCTRETime.info(visionIOInputs.lastResult.timestamp());
    logs.connected.info(visionIOInputs.connected);
    logs.medianLatency.info(visionIOInputs.medianLatency);
    logs.medianUpdateTime.info(visionIOInputs.medianUpdateTime);

    logs.lastTimestampCTRETimeSuccessful.info(lastSuccessfullyProcessedResultTimeStampCTRETime);

    // Logging
    logs.cameraPose.info(robotPose.transformBy(RobotToCamera));

    var fieldsToLog = loggedFields;
    var status      = fieldsToLog.status();
    if (!status.success) {
      if (status == lastStatus) {
        // skip logging details of unsuccessful results that repeat
        return;
      }

      double minSeparationInMilliseconds = 100.0 / 1000.0;

      if (currentTime - lastSuccessfullyProcessedResultTimeStampCTRETime < minSeparationInMilliseconds) {
        // skip logging details of unsuccessful results that are too close to a
        // successful report.
        return;
      }
    }

    lastStatus = status;
    logs.status.info(status.name());
    logs.successfulResult.info(status.success);

    if (!status.success) {
      return;
    }

    double[] seenTagsArray = new double[fieldsToLog.seenTags().length];

    for (int i = 0; i < fieldsToLog.seenTags().length; i++) {
      seenTagsArray[i] = fieldsToLog.seenTags()[i];
    }

    if (status.success) {
      Pose3d tagPose = aprilTagLayout.getTagPose(TAG_ID).get();
      if (tagPose != null) {
        Pose3d relativePose = fieldsToLog.robotPose3d().relativeTo(tagPose);
        logs.distanceToTag.info(relativePose);
        logs.yawToTag.info(Math.toDegrees(relativePose.getRotation().getZ()));
      }
    }

    logs.calculatedRobotPose3d.info(fieldsToLog.robotPose3d());
    logs.calculatedRobotPose2d.info(fieldsToLog.robotPose3d().toPose2d());
    logs.numSeenTargets.info(fieldsToLog.numSeenTargets());
    logs.distanceFromExistingPoseEstimate.info(fieldsToLog.distanceFromExistingPoseEstimate());
    logs.averageDistanceFromTags.info(fieldsToLog.averageDistanceFromTags());
    logs.tagAmbiguity.info(fieldsToLog.tagAmbiguity());
    logs.xyStandardDeviation.info(fieldsToLog.xyStandardDeviation());
    logs.thetaStandardDeviation.info(fieldsToLog.thetaStandardDeviation());
    logs.seenTargetsIDs.info(seenTagsArray);
  }

  public Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult cameraResult) {
    var result = photonPoseEstimator.estimateCoprocMultiTagPose(cameraResult);
    if (result.isEmpty()) {
      result = photonPoseEstimator.estimateClosestToCameraHeightPose(cameraResult);
    }

    return result;
  }

  public void updateInputs() {
    visionIO.updateInputs(visionIOInputs);
  }

  public boolean isCameraConnected() {
    return visionIOInputs.connected;
  }

  public VisionIO.ResultWithTimestamp getLastResult() {
    synchronized (visionIOInputs) {
      return visionIOInputs.lastResult;
    }
  }
}
