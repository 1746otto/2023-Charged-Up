package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  static PhotonCamera turretCamera;
  static PhotonPipelineResult lastResult;
  static PhotonTrackedTarget bestTarget;

  public Vision() {
    turretCamera = new PhotonCamera("rotto");
  }

  public static void getLastResult() {
    lastResult = turretCamera.getLatestResult();
    bestTarget = lastResult.getBestTarget();
  }

  public static double getxOffset() {
    return NetworkTableInstance.getDefault().getEntry("targetPixelsX").getDouble(0) / 160.0 * 29.8;
  }
}
