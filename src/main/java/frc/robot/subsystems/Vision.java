package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
  static PhotonCamera lottoCamera;
  static PhotonPipelineResult lottoLastResult;
  static PhotonTrackedTarget lottoBestTarget;
  static PhotonCamera rottoCamera;
  static PhotonPipelineResult rottoLastResult;
  static PhotonTrackedTarget rottoBestTarget;
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
    } catch (Exception e) {
    } ;
    lottoCamera = new PhotonCamera("lotto");
    rottoCamera = new PhotonCamera("rotto");
    new Thread(() -> {
      while (true) {
        List<Pose3d> poses = new ArrayList<>();
        Vision.getLastResult();
        if (lottoLastResult.hasTargets()) {
          SmartDashboard.putString("Lotto April Tag Rotation", Double.toString(
              lottoBestTarget.getBestCameraToTarget().getRotation().getX() / Math.PI * 180)
              + ", "
              + Double.toString(
                  lottoBestTarget.getBestCameraToTarget().getRotation().getY() / Math.PI * 180)
              + ", " + Double.toString(
                  lottoBestTarget.getBestCameraToTarget().getRotation().getZ() / Math.PI * 180));
          for (PhotonTrackedTarget target : Vision.lottoLastResult.targets) {
            poses.add(tags.getTagPose(target.getFiducialId()).get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToLotto.inverse()));
          }
        }
        if (rottoLastResult.hasTargets()) {
          SmartDashboard
              .putString("Rotto April Tag Rotation",
                  Double
                      .toString(tags.getTagPose(rottoBestTarget.getFiducialId()).get()
                          .transformBy(rottoBestTarget.getBestCameraToTarget().inverse())
                          .transformBy(robotToRotto.inverse()).getRotation().getX() / Math.PI * 180)
                      + ", "
                      + Double.toString(tags.getTagPose(rottoBestTarget.getFiducialId()).get()
                          .transformBy(rottoBestTarget.getBestCameraToTarget().inverse())
                          .transformBy(robotToRotto.inverse()).getRotation().getY() / Math.PI * 180)
                      + ", "
                      + Double.toString(tags.getTagPose(rottoBestTarget.getFiducialId()).get()
                          .transformBy(rottoBestTarget.getBestCameraToTarget().inverse())
                          .transformBy(robotToRotto.inverse()).getRotation().getZ() / Math.PI
                          * 180));
          for (PhotonTrackedTarget target : Vision.rottoLastResult.targets) {
            poses.add(tags.getTagPose(target.getFiducialId()).get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToRotto.inverse()));
          }
        }
        PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(null, null, lottoCamera, null);
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
    lottoLastResult = lottoCamera.getLatestResult();
    lottoBestTarget = lottoLastResult.getBestTarget();
    rottoLastResult = rottoCamera.getLatestResult();
    rottoBestTarget = rottoLastResult.getBestTarget();
    SmartDashboard.putNumber("Get state time", Timer.getFPGATimestamp() - start);
  }
}
