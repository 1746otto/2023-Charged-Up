package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  static PhotonCamera turretCamera;
  static PhotonPipelineResult lastResult;
  static PhotonTrackedTarget bestTarget;

  public Vision() {
    turretCamera = new PhotonCamera("lotto");
  }

  public static void getLastResult() {
    lastResult = turretCamera.getLatestResult();
    bestTarget = lastResult.getBestTarget();
  }
}
