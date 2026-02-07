package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  record ResultWithTimestamp(PhotonPipelineResult result, double timestamp) {
  }

  class Inputs {
    ResultWithTimestamp lastResult = new ResultWithTimestamp(null, -1.0);
    boolean             connected;
    double              medianLatency;
    double              medianUpdateTime;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(Inputs inputs) {
  }

  default PhotonCamera getCamera() {
    System.out.println("------default getCamera() == null  ------------");
    return null;
  }

  default void updateTimeMedian(double latency, double timeSinceLastUpdate) {
    // medianLatency = latencyMedianFilter.calculate(latency);
    // medianUpdateTime = updateTimeMedianFilter.calculate(timeSinceLastUpdate);
  }
}
