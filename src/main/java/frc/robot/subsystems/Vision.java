package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
  static PhotonCamera turretCamera;
  static PhotonPipelineResult lastResult;
  static PhotonTrackedTarget bestTarget;
  private Pose3d lottoEstimatedRobotPose = new Pose3d();
  private Pose3d rottoEstimatedRobotPose = new Pose3d();
  static Pose3d robotEstimatedPose = new Pose3d();
  private AprilTagFieldLayout tags;
  // +X is forward, Y is left and right, +Z is up
  public static Transform3d robotToLotto =
      new Transform3d(new Translation3d(0.5, 0.5, 0.5), new Rotation3d(0, -Math.PI / 9.0, -29.8));
  public static Transform3d robotToRotto =
      new Transform3d(new Translation3d(0.5, -0.5, 0.5), new Rotation3d(0, -Math.PI / 9.0, 29.8));

  public Vision() {
    try {
    tags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    }
    catch(Exception e) {};
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
                      bestTarget.getBestCameraToTarget().
                      getRotation().getZ() / Math.PI * 180));
          for (PhotonTrackedTarget target : Vision.lastResult.targets) {
            target.getBestCameraToTarget();
          }
        PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(null, null, turretCamera, null);
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
