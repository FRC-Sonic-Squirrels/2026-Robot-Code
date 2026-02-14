package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import frc.robot.configs.RobotConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.EnumSet;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
  private double               lastTimestamp = 0;
  private PhotonPipelineResult lastResult    = new PhotonPipelineResult();

  private final Supplier<Pose2d> poseSupplier;

  private final AprilTagFieldLayout layout;

  private final PhotonCamera    camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  public VisionIOSim(RobotConfig config, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera, String networkName) {
    this.poseSupplier = poseSupplier;
    this.layout       = config.getAprilTagFieldLayout();

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(128.2));
    cameraProp.setCalibError(0.01, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(10);

    this.camera    = new PhotonCamera(networkName);
    this.visionSim = new VisionSystemSim(networkName);
    visionSim.addAprilTags(layout);
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    /*
     * based on
     * https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-
     * change.html#listening-for-changes and
     * https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java
     * /frc/robot/subsystems/vision /VisionIOPhotonVision.java
     */
    var targetPoseSub = inst.getTable("/photonvision/" + networkName)
        .getStructTopic("targetPose", Transform3d.struct).subscribe(null);

    inst.addListener(targetPoseSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
      for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        double timestamp = result.getTimestampSeconds();
        synchronized (VisionIOSim.this) {
          lastTimestamp = timestamp;
          lastResult    = result;
        }
      }
    });
  }

  @Override
  public synchronized void updateInputs(Inputs inputs) {
    visionSim.update(poseSupplier.get());

    inputs.lastResult = new VisionIO.ResultWithTimestamp(this.lastResult, this.lastTimestamp);
    inputs.connected  = camera.isConnected();
  }

  @Override
  public PhotonCamera getCamera() {
    return camera;
  }
}
