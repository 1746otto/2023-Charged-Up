package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix.sensors.Pigeon2;
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
  private Pose3d temp;
  private AprilTagFieldLayout tags;
  private DoubleSupplier gyroVal;
  private double tolerance = 3000;
  private double maxDist = 1000;
  private Pigeon2 gyro = new Pigeon2(5, "CANivore");
  // +X is forward, Y is left and right, +Z is up
  public static Transform3d robotToLotto = new Transform3d(
      new Translation3d(0.219075, 0.231775, 0.676275), new Rotation3d(0, Math.PI / 9.0, -29.8));
  public static Transform3d robotToRotto =
      new Transform3d(new Translation3d(0.5, -0.5, 0.5), new Rotation3d(0, Math.PI / 9.0, 29.8));

  public Vision(DoubleSupplier yawSupplier) {
    try {
      tags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      SmartDashboard.putString("fail", "fail");
    } ;
    // gyroVal = yawSupplier;
    gyroVal = () -> {
      return gyro.getYaw();
    };
    lottoCamera = new PhotonCamera("lotto");
    rottoCamera = new PhotonCamera("rotto");
    new Thread(() -> {
      while (true) {
        List<Pose3d> poses = new ArrayList<>();
        Vision.getLastResult();
        if (lottoLastResult.hasTargets()) {
          SmartDashboard.putBoolean("lotto has target", true);
          for (PhotonTrackedTarget target : Vision.lottoLastResult.targets) {
            if (target.getFiducialId() > 8 || target.getFiducialId() <= 0) {
              continue;
            }
            temp = tags.getTagPose(target.getFiducialId()).get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToLotto.inverse());
            if (target.getBestCameraToTarget().getTranslation().getNorm() < maxDist) {
              if (!(gyroVal.getAsDouble() - tolerance < temp.getRotation().getZ()
                  && temp.getRotation().getZ() < gyroVal.getAsDouble() + tolerance)) {
                temp = tags.getTagPose(target.getFiducialId()).get()
                    .transformBy(target.getAlternateCameraToTarget().inverse())
                    .transformBy(robotToLotto.inverse());
                if (gyroVal.getAsDouble() - tolerance < temp.getRotation().getZ()
                    && temp.getRotation().getZ() < gyroVal.getAsDouble() + tolerance) {
                  poses.add(temp);
                }
              } else {
                SmartDashboard.putBoolean("addedValue", true);
                poses.add(temp);
              }
            }
          }
        } else {

          SmartDashboard.putBoolean("lotto has target", false);
        }
        if (rottoLastResult.hasTargets()) {

          SmartDashboard.putBoolean("rotto has target", true);
          for (PhotonTrackedTarget target : Vision.rottoLastResult.targets) {
            if (target.getFiducialId() > 8 || target.getFiducialId() <= 0) {
              continue;
            }
            temp = tags.getTagPose(target.getFiducialId()).get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToRotto.inverse());
            if (target.getBestCameraToTarget().getTranslation().getNorm() < maxDist) {
              if (!(gyroVal.getAsDouble() - tolerance < temp.getRotation().getZ()
                  && temp.getRotation().getZ() < gyroVal.getAsDouble() + tolerance)) {
                temp = tags.getTagPose(target.getFiducialId()).get()
                    .transformBy(target.getAlternateCameraToTarget().inverse())
                    .transformBy(robotToRotto.inverse());
                if (gyroVal.getAsDouble() - tolerance < temp.getRotation().getZ()
                    && temp.getRotation().getZ() < gyroVal.getAsDouble() + tolerance) {
                  poses.add(temp);
                }
              } else {
                poses.add(temp);
              }
            }
          }
        } else {
          SmartDashboard.putBoolean("rotto has target", false);
        }
        robotEstimatedPose = new Pose3d();
        for (Pose3d pose : poses) {
          robotEstimatedPose = robotEstimatedPose.plus(new Transform3d(new Pose3d(), pose));
        }
        robotEstimatedPose = robotEstimatedPose.div(poses.size());
        SmartDashboard.putString("pose",
            robotEstimatedPose.getTranslation().toString() + ", "
                + Double.toString(robotEstimatedPose.getRotation().getX() / Math.PI * 180) + ", "
                + Double.toString(robotEstimatedPose.getRotation().getY() / Math.PI * 180) + ", "
                + Double.toString(robotEstimatedPose.getRotation().getZ() / Math.PI * 180));
        poses.clear();
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          SmartDashboard.putString("fail", e.getMessage());
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
