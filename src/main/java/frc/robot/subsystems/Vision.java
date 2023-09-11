package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
  static PhotonCamera turretCamera;
  static PhotonPipelineResult lastResult;
  static PhotonTrackedTarget bestTarget;

  public Vision() {
    turretCamera = new PhotonCamera("rotto");
    new Thread(() -> {
      while (true) {
        Vision.getLastResult();
        if (lastResult.hasTargets())
          SmartDashboard.putString("April Tag Rotation",
              Double
                  .toString(bestTarget.getBestCameraToTarget().getRotation().getX() / Math.PI * 180)
                  + ", "
                  + Double.toString(
                      bestTarget.getBestCameraToTarget().getRotation().getY() / Math.PI * 180)
                  + ", " + Double.toString(
                      bestTarget.getBestCameraToTarget().getRotation().getZ() / Math.PI * 180));
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }).start();
  }

  public static void getLastResult() {
    double start = Timer.getFPGATimestamp();
    lastResult = turretCamera.getLatestResult();
    bestTarget = lastResult.getBestTarget();
    SmartDashboard.putNumber("Get state time", Timer.getFPGATimestamp() - start);
  }
}
