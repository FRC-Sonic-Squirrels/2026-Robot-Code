package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
  public static final String ROOT_TABLE = "Vision";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  public static class VisibleTagsLogs extends AutoRegisterLoggers {
    public LoggerEntry.StructArray<Pose3d> rawAllVisibleTags;
    public LoggerEntry.StructArray<Pose3d> tagsActuallyUsedInPoseEstimation;
    public LoggerEntry.StructArray<Pose3d> posesFedToPoseEstimator3D;
    public LoggerEntry.StructArray<Pose2d> posesFedToPoseEstimator2D;
  }

  public static class Logs extends AutoRegisterLoggers {
    public VisibleTagsLogs visibleTags;

    public LoggerEntry.StructArray<Pose3d> allAprilTags3D;
    public LoggerEntry.Bool                useVision;
    public LoggerEntry.Bool                useMaxDistanceAwayFromExistingEstimate;

    public LoggerEntry.Integer rejectByGyro;
    public LoggerEntry.Decimal rejectByGyroError;
    public LoggerEntry.Integer rejectByDistance;
    public LoggerEntry.Decimal rejectByDistanceError;
    public LoggerEntry.Integer configTagAmount;

    public LoggerEntry.Struct<Pose3d> poseEstimatorError;
    public LoggerEntry.Decimal        poseEstimatorErrorDistance;
    public LoggerEntry.Decimal        poseEstimatorErrorPitch;
    public LoggerEntry.Decimal        poseEstimatorErrorYaw;
    public LoggerEntry.Decimal        poseEstimatorErrorRoll;
  }

  public static final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Tunable numbers

  protected static final TunableNumberGroup group = new TunableNumberGroup("Vision");

  private static final LoggedTunableNumber thetaStdDevCoefficient = group.build("thetaStdDevCoefficient", 0.075);
  private static final LoggedTunableNumber xyStdDevCoefficient    = group.build("xyStdDevCoefficient", 0.075);

  private static final LoggedTunableNumber maxSingleTargetAmbiguity = group.build("MaxSingleTargetAmbiguity", 0.2);

  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters = group
      .build("MaxValidDistanceFromCurrentEstimateMeters", 30.0);

  private static final LoggedTunableNumber gyroFilteringToleranceDegrees = group.build("GyroFilteringToleranceDegrees",
      20.0);

  private static final LoggedTunableNumber zHeightToleranceMeters = group.build("zHeightToleranceMeters", 0.5);

  private static final LoggedTunableNumber pitchAndRollToleranceDegrees = group.build("pitchToleranceDegrees", 10.0);

  private final List<VisionModule> visionModules = new ArrayList<>();

  private final AprilTagFieldLayout aprilTagLayout;

  private boolean useVisionForPoseEstimation             = true;
  private boolean useMaxDistanceAwayFromExistingEstimate = true;
  private boolean useGyroBasedFilteringForVision         = true;

  private final HashMap<Integer, Double> lastTagDetectionTimes    = new HashMap<>();
  private final List<Short>              tagsUsedInPoseEstimation = new ArrayList<>();

  private final List<Pose3d> posesFedToPoseEstimator3D = new ArrayList<>();
  private final List<Pose2d> posesFedToPoseEstimator2D = new ArrayList<>();

  private final List<TimestampedVisionUpdate> allTimestampedVisionUpdates = new ArrayList<>();

  private final Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer;
  private final Supplier<Pose2d>                        poseEstimatorPoseSupplier;
  private final Supplier<Rotation2d>                    currentGyroBasedRobotRotationSupplier;
  private int                                           useMaxDistanceAwayFromExistingEstimateCount;
  private int                                           useGyroBasedFilteringForVisionCount;

  public Vision(AprilTagFieldLayout aprilTagLayout, Supplier<Pose2d> poseEstimatorPoseSupplier,
      Supplier<Rotation2d> currentGyroBasedRobotRotationSupplier,
      Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer,
      VisionModuleConfiguration... visionModuleConfigs) {

    this.visionEstimatesConsumer               = visionEstimatesConsumer;
    this.poseEstimatorPoseSupplier             = poseEstimatorPoseSupplier;
    this.currentGyroBasedRobotRotationSupplier = currentGyroBasedRobotRotationSupplier;
    this.aprilTagLayout                        = aprilTagLayout;

    for (int i = 0; i < visionModuleConfigs.length; i++) {
      visionModules.add(new VisionModule(logs.group, visionModuleConfigs[i], aprilTagLayout, i));
    }

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));

    Pose3d[] allTagsArray = new Pose3d[aprilTagLayout.getTags().size()];
    var      allTagsList  = aprilTagLayout.getTags();
    for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
      allTagsArray[i] = allTagsList.get(i).pose;
    }
    logs.allAprilTags3D.info(allTagsArray);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // update all inputs
      for (VisionModule module : visionModules) {
        module.updateInputs();
      }

      boolean processVision = true;
      if (!useVisionForPoseEstimation) {
        processVision = false;
        setAllVisionModuleUnsuccessfulStatus(VisionResultStatus.VISION_DISABLED);
      }

      var currentTime = Utils.getCurrentTimeSeconds();

      // process vision
      if (processVision) {
        for (VisionModule visionModule : visionModules) {
          visionModule.loggedFields = processVision(visionModule, currentTime);
        }
      }

      // send successful results to pose estimator
      if (!allTimestampedVisionUpdates.isEmpty()) {
        visionEstimatesConsumer.accept(allTimestampedVisionUpdates);
        allTimestampedVisionUpdates.clear();
      }

      // log all vision module's logged fields
      var robotPose = new Pose3d(poseEstimatorPoseSupplier.get());
      for (VisionModule visionModule : visionModules) {
        visionModule.log(robotPose, currentTime);
      }

      // activate alerts if camera is not connected
      for (VisionModule module : visionModules) {
        module.missingCameraAlert.set(module.isCameraConnected());
      }

      // logging all visible tags
      List<Pose3d> allSeenTags = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (detectionEntry.getValue() == Timer.getFPGATimestamp()) {
          var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
          if (tagPose.isPresent()) {
            allSeenTags.add(tagPose.get());
          }
        }
      }
      logs.visibleTags.rawAllVisibleTags.info(allSeenTags.toArray(new Pose3d[0]));

      // logging tags actually used in pose estimation
      Pose3d[] tagsUsedInPoseEstimationPoses = new Pose3d[tagsUsedInPoseEstimation.size()];
      for (int i = 0; i < tagsUsedInPoseEstimation.size(); i++) {
        tagsUsedInPoseEstimationPoses[i] = aprilTagLayout.getTagPose(tagsUsedInPoseEstimation.get(i)).get();
      }

      logs.visibleTags.tagsActuallyUsedInPoseEstimation.info(tagsUsedInPoseEstimationPoses);
      logs.visibleTags.posesFedToPoseEstimator3D.info(posesFedToPoseEstimator3D.toArray(new Pose3d[0]));
      logs.visibleTags.posesFedToPoseEstimator2D.info(posesFedToPoseEstimator2D.toArray(new Pose2d[0]));

      if (posesFedToPoseEstimator3D.size() > 1) {
        Pose3d lastError         = posesFedToPoseEstimator3D.get(0).relativeTo(posesFedToPoseEstimator3D.get(1));
        double lastErrorDistance = lastError.getTranslation().getNorm();
        double lastErrorPitch    = Math.toDegrees(lastError.getRotation().getY());
        double lastErrorYaw      = Math.toDegrees(lastError.getRotation().getZ());
        double lastErrorRoll     = Math.toDegrees(lastError.getRotation().getX());

        logs.poseEstimatorError.info(lastError);
        logs.poseEstimatorErrorDistance.info(lastErrorDistance);
        logs.poseEstimatorErrorPitch.info(lastErrorPitch);
        logs.poseEstimatorErrorYaw.info(lastErrorYaw);
        logs.poseEstimatorErrorRoll.info(lastErrorRoll);
      }

      tagsUsedInPoseEstimation.clear();
      posesFedToPoseEstimator3D.clear();
      posesFedToPoseEstimator2D.clear();

      logs.useVision.info(useVisionForPoseEstimation);
      logs.useMaxDistanceAwayFromExistingEstimate.info(useMaxDistanceAwayFromExistingEstimate);
    }
  }

  public VisionResultLoggedFields processVision(VisionModule visionModule, double currentTime) {
    Pose2d prevEstimatedRobotPose = poseEstimatorPoseSupplier.get();

    var                  lastCameraResult               = visionModule.getLastResult();
    var                  currentResultTimeStampCTRETime = lastCameraResult.timestamp();
    PhotonPipelineResult pipelineResult                 = lastCameraResult.result();

    if (visionModule.lastSuccessfullyProcessedResultTimeStampCTRETime >= currentResultTimeStampCTRETime) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.NOT_A_NEW_RESULT);
    }

    if (currentResultTimeStampCTRETime > currentTime) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.TIMESTAMP_IN_FUTURE);
    }

    visionModule.lastSuccessfullyProcessedResultTimeStampCTRETime = currentResultTimeStampCTRETime;

    // remove any tags that are not part of the field layout
    var         timeStampCameraResult = pipelineResult.getTimestampSeconds();
    List<Short> cleanTargets          = new ArrayList<>();

    for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
      int              fiducialId = target.getFiducialId();
      Optional<Pose3d> optTagPose = aprilTagLayout.getTagPose(fiducialId);
      if (optTagPose.isEmpty()) {
        continue;
      }

      cleanTargets.add((short) target.getFiducialId());
      lastTagDetectionTimes.put(fiducialId, timeStampCameraResult);
    }

    var numTargetsSeen = cleanTargets.size();
    if (numTargetsSeen == 0) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.NO_TARGETS_VISIBLE);
    }

    if (numTargetsSeen == 1) {
      PhotonTrackedTarget singularTag = pipelineResult.getTargets().get(0);

      if (!isValidTarget(singularTag)) {
        return (singularTag.getPoseAmbiguity() > maxSingleTargetAmbiguity.get())
            ? VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INVALID_TAG_AMBIGUITY_TOO_HIGH)
            : VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INVALID_TAG);
      }
    } else {
      PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();
      for (PhotonTrackedTarget result : pipelineResult.getTargets()) {
        if (result.area > bestTarget.area) {
          bestTarget = result;
        }
      }
    }

    var photonPoseEstimatorOptionalResult = visionModule.estimateRobotPose(pipelineResult);
    if (photonPoseEstimatorOptionalResult.isEmpty()) {
      return VisionResultLoggedFields
          .unsuccessfulResult(VisionResultStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY);
    }

    // used to log if the multi tag failed
    EstimatedRobotPose photonPoseEstimatorResult = photonPoseEstimatorOptionalResult.get();
    var                multiTagSuccess           = photonPoseEstimatorResult.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    if (multiTagSuccess) {
      // only include tags used by PNP
      cleanTargets = pipelineResult.getMultiTagResult().get().fiducialIDsUsed;
    }

    var    newCalculatedRobotPose   = photonPoseEstimatorResult.estimatedPose;
    Pose2d newCalculatedRobotPose2d = newCalculatedRobotPose.toPose2d();
    if (GeometryUtil.isPoseOutsideField(newCalculatedRobotPose2d)) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD);
    }

    Rotation3d newCalculatedRobotPoseRotation = newCalculatedRobotPose.getRotation();
    double     posePitch                      = newCalculatedRobotPoseRotation.getX();
    double     poseRoll                       = newCalculatedRobotPoseRotation.getY();
    double     poseYaw                        = newCalculatedRobotPoseRotation.getZ();

    if (useGyroBasedFilteringForVision) {
      var gyroYaw  = currentGyroBasedRobotRotationSupplier.get();
      var yawDelta = GeometryUtil.optimizeRotation(poseYaw - gyroYaw.getRadians());

      var absError = Math.toDegrees(Math.abs(yawDelta));
      logs.rejectByGyroError.info(absError);

      if (absError > gyroFilteringToleranceDegrees.get()) {
        logs.rejectByGyro.info(++useGyroBasedFilteringForVisionCount);
        return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION);
      }
    }

    if (Math.abs(newCalculatedRobotPose.getZ()) >= zHeightToleranceMeters.get()) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.Z_HEIGHT_BAD);
    }

    var pitchAndRollToleranceValue = pitchAndRollToleranceDegrees.get();
    if (Math.abs(poseRoll) >= pitchAndRollToleranceValue || Math.abs(posePitch) >= pitchAndRollToleranceValue) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.PITCH_OR_ROLL_BAD);
    }

    double tagAmbiguity  = 0.0;
    double totalDistance = 0.0;
    double averageDistanceFromTags;

    if (multiTagSuccess) {

      // use average distance for PNP with multiple tags
      for (int tag : cleanTargets) {
        totalDistance += aprilTagLayout.getTagPose(tag).get().getTranslation()
            .getDistance(newCalculatedRobotPose.getTranslation());
      }

      averageDistanceFromTags = totalDistance / (double) cleanTargets.size();
    } else {
      // Use single tag distance for all other methods
      totalDistance = aprilTagLayout.getTagPose(pipelineResult.getBestTarget().getFiducialId()).get().getTranslation()
          .getDistance(newCalculatedRobotPose.getTranslation());

      averageDistanceFromTags = totalDistance;
    }

    var distanceFromExistingPoseEstimate = GeometryUtil.getDist(prevEstimatedRobotPose, newCalculatedRobotPose2d);

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate > (maxValidDistanceAwayFromCurrentEstimateMeters.get()
            * numTargetsSeen))) {
      logs.rejectByDistance.info(++useMaxDistanceAwayFromExistingEstimateCount);
      logs.rejectByDistanceError.info(distanceFromExistingPoseEstimate);
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE);
    }

    var averageDistanceFromTagsSquared = averageDistanceFromTags * averageDistanceFromTags;
    var xyStandardDeviation            = xyStdDevCoefficient.get() * averageDistanceFromTagsSquared / numTargetsSeen;
    var thetaStandardDeviation         = thetaStdDevCoefficient.get() * averageDistanceFromTagsSquared / numTargetsSeen;

    var visionUpdate            = new PoseEstimator.VisionUpdate(newCalculatedRobotPose2d, xyStandardDeviation,
        xyStandardDeviation, thetaStandardDeviation);
    var timestampedVisionUpdate = new PoseEstimator.TimestampedVisionUpdate(cleanTargets,
        currentResultTimeStampCTRETime, visionUpdate);

    allTimestampedVisionUpdates.add(timestampedVisionUpdate);

    posesFedToPoseEstimator3D.add(newCalculatedRobotPose);
    posesFedToPoseEstimator2D.add(newCalculatedRobotPose2d);
    tagsUsedInPoseEstimation.addAll(cleanTargets);

    VisionResultStatus status;
    if (numTargetsSeen == 1 || !multiTagSuccess) {
      status = VisionResultStatus.SUCCESSFUL_SINGLE_TAG;
    } else {
      status = VisionResultStatus.SUCCESSFUL_MULTI_TAG;
    }

    return new VisionResultLoggedFields(status, numTargetsSeen, tagAmbiguity, averageDistanceFromTags,
        distanceFromExistingPoseEstimate, xyStandardDeviation, thetaStandardDeviation, newCalculatedRobotPose,
        timestampedVisionUpdate.tags().toArray(new Short[timestampedVisionUpdate.tags().size()]));
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1 && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() <= maxSingleTargetAmbiguity.get()
        && aprilTagLayout.getTagPose(target.getFiducialId()).isPresent();
  }

  public void useMaxDistanceAwayFromExistingEstimate(boolean value) {
    useMaxDistanceAwayFromExistingEstimate = value;
  }

  public void setAllVisionModuleUnsuccessfulStatus(VisionResultStatus status) {
    for (VisionModule cameraPackage : visionModules) {
      cameraPackage.loggedFields = VisionResultLoggedFields.unsuccessfulResult(status);
    }
  }

  public void setUsingVision(boolean value) {
    useVisionForPoseEstimation = value;
  }

  public void useGyroBasedFilteringForVision(boolean value) {
    useGyroBasedFilteringForVision = value;
  }

  public boolean isUsingVision() {
    return useVisionForPoseEstimation;
  }

  public static void restartPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartProgram");
  }

  public static void rebootPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartDevice");
  }

  public static void sendPhotonVisionCommand(String ipString, String command) {
    try {
      HttpClient  httpClient = HttpClient.newHttpClient();
      HttpRequest request    = HttpRequest.newBuilder()
          .uri(new URI("http://" + ipString + ":5800/api/utils/" + command)).POST(HttpRequest.BodyPublishers.noBody())
          .build();
      httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString());
    } catch (Exception ignored) {
    }
  }
}
