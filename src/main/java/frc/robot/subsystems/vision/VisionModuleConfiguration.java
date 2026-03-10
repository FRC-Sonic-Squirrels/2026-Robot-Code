package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.configs.RobotConfig;

import java.util.function.Supplier;

public class VisionModuleConfiguration {
  public final VisionIO    visionIO;
  public final String      logName;
  public final Transform3d robotToCamera;

  private VisionModuleConfiguration(VisionIO visionIO, String logName, Transform3d robotToCamera) {
    this.visionIO      = visionIO;
    this.logName       = logName;
    this.robotToCamera = robotToCamera;
  }

  public static VisionModuleConfiguration build(String name, CameraConfiguration camera) {
    var visionIO = new VisionIOPhotonVision(name);

    return new VisionModuleConfiguration(visionIO, name, camera.toTransform3d());
  }

  public static VisionModuleConfiguration buildSim(String name, CameraConfiguration camera, RobotConfig config,
      Supplier<Pose2d> poseSupplier) {
    var visionIO = new VisionIOSim(config, poseSupplier, camera.toTransform3d(), name);
    return new VisionModuleConfiguration(visionIO, name, camera.toTransform3d());
  }

  public static VisionModuleConfiguration buildReplayStub(String name, CameraConfiguration camera) {
    var visionIO = new VisionIO() {
    };
    return new VisionModuleConfiguration(visionIO, name, camera.toTransform3d());
  }
}
