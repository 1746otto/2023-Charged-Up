package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// It's ugly but it works
public class Vision {
  static PhotonCamera lottoCamera;
  static PhotonPipelineResult lottoLastResult;
  static PhotonTrackedTarget lottoBestTarget;
  static PhotonCamera rottoCamera;
  static PhotonPipelineResult rottoLastResult;
  static PhotonTrackedTarget rottoBestTarget;
  static Pose3d lottoEstimatedPose = new Pose3d();
  static Pose3d rottoEstimatedPose = new Pose3d();
  static Pose3d robotEstimatedPose = new Pose3d();
  private Pose3d temp;
  private AprilTagFieldLayout tags;
  private DoubleSupplier gyroVal;
  private BiConsumer<Pose2d, Double> addVisionMeasurement;
  private double tolerance = 3000;
  private double maxDist = 1000;
  private Thread somethingThread;
  private int iTemp = 0;
  // +X is forward, Y is left and right, +Z is up
  public static Transform3d robotToLotto =
      new Transform3d(new Translation3d(0.203831, 0.22993, 0.6292135),
          new Rotation3d(Units.degreesToRadians(-9.78636928303), Units.degreesToRadians(20),
              Units.degreesToRadians(-29.8)));
  public static Transform3d robotToRotto =
      new Transform3d(new Translation3d(0.203831, -0.22993, 0.6292135),
          new Rotation3d(Units.degreesToRadians(9.78636928303), Units.degreesToRadians(20),
              Units.degreesToRadians(29.8)));

  public Vision(DoubleSupplier yawSupplier, BiConsumer<Pose2d, Double> addPose) {
    try {
      tags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      SmartDashboard.putString("fail", "fail");
    } ;
    gyroVal = yawSupplier;
    addVisionMeasurement = addPose;
    lottoCamera = new PhotonCamera("lotto");
    rottoCamera = new PhotonCamera("rotto");
    somethingThread = new Thread(() -> {
      while (true) {
        try {
          SmartDashboard.putNumber("iteration", iTemp);
          List<Pose3d> poses = new ArrayList<>();
          List<Pose3d> allPoses = new ArrayList<>();
          Vision.getLastResult();
          if (lottoLastResult.hasTargets()) {
            SmartDashboard.putBoolean("lotto has target", true);
            lottoLastResult.getTimestampSeconds();
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
            for (Pose3d pose : poses) {
              lottoEstimatedPose = robotEstimatedPose.plus(new Transform3d(new Pose3d(), pose));
            }

            lottoEstimatedPose = lottoEstimatedPose.div(poses.size());
            allPoses.addAll(poses);
            poses.clear();

            SmartDashboard.putNumber("Lotto Timestamp", lottoLastResult.getTimestampSeconds());
            SmartDashboard.putNumber("Lotto calculated latency",
                Timer.getFPGATimestamp() - lottoLastResult.getTimestampSeconds());
            if (Timer.getFPGATimestamp() - lottoLastResult.getTimestampSeconds() >= 1.5) {
              SmartDashboard.putBoolean("Lotto 1.5+ latency issuen", true);
            } else {
              // Normally WPILIB would handle this for us or at least throw an error, however there
              // is an issue where it doesn't end up throwing errors. Instead it just freezes the
              // thread. So we make sure not to pass in any values that are older than WPILib's pose
              // estimator can handle.
              addVisionMeasurement.accept(lottoEstimatedPose.toPose2d(),
                  lottoLastResult.getTimestampSeconds());
            }
            SmartDashboard.putString("lotto pose", lottoEstimatedPose.getTranslation().toString()
                + ", " + Double.toString(lottoEstimatedPose.getRotation().getX() / Math.PI * 180)
                + ", " + Double.toString(lottoEstimatedPose.getRotation().getY() / Math.PI * 180)
                + ", " + Double.toString(lottoEstimatedPose.getRotation().getZ() / Math.PI * 180));
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
            rottoEstimatedPose = new Pose3d();
            for (Pose3d pose : poses) {
              rottoEstimatedPose = rottoEstimatedPose.plus(new Transform3d(new Pose3d(), pose));
            }
            rottoEstimatedPose.div(poses.size());
            allPoses.addAll(poses);
            poses.clear();
            SmartDashboard.putNumber("Rotto Timestamp", rottoLastResult.getTimestampSeconds());
            SmartDashboard.putNumber("Rotto calculated latency",
                Timer.getFPGATimestamp() - rottoLastResult.getTimestampSeconds());
            if (Timer.getFPGATimestamp() - rottoLastResult.getTimestampSeconds() >= 1.5) {
              SmartDashboard.putBoolean("Rotto 1.5+ latency issuen", true);
            } else {
              // Same issue as with this as on lotto.
              addVisionMeasurement.accept(rottoEstimatedPose.toPose2d(),
                  rottoLastResult.getTimestampSeconds());
            }
            SmartDashboard.putString("rotto pose", rottoEstimatedPose.getTranslation().toString()
                + ", " + Double.toString(rottoEstimatedPose.getRotation().getX() / Math.PI * 180)
                + ", " + Double.toString(rottoEstimatedPose.getRotation().getY() / Math.PI * 180)
                + ", " + Double.toString(rottoEstimatedPose.getRotation().getZ() / Math.PI * 180));
          } else {
            SmartDashboard.putBoolean("rotto has target", false);
          }

          SmartDashboard.putString("arraylist", allPoses.toString());
          if (allPoses.size() > 0) {
            robotEstimatedPose = new Pose3d();
            for (Pose3d pose : allPoses) {
              robotEstimatedPose = robotEstimatedPose.plus(new Transform3d(new Pose3d(), pose));
            }
            robotEstimatedPose = robotEstimatedPose.div(allPoses.size());
          }
          SmartDashboard.putString("pose",
              robotEstimatedPose.getTranslation().toString() + ", "
                  + Double.toString(robotEstimatedPose.getRotation().getX() / Math.PI * 180) + ", "
                  + Double.toString(robotEstimatedPose.getRotation().getY() / Math.PI * 180) + ", "
                  + Double.toString(robotEstimatedPose.getRotation().getZ() / Math.PI * 180));
          poses.clear();
          try {
            Thread.sleep(10);
          } catch (InterruptedException e) {
            // Occasionally Thread.sleep fails. For some reason when the message is queried either
            // with getMessage() or toString(), it returns null. I am unsure if the issue is that
            // the error is not giving a reason or something like that or if its just another WPILib
            // sets the errors to null. Either way, when queried the error return null, so on
            // passing this SmartDashboard causes an exception to be thrown.
            // System.out.println("sleep: " + e.toString()); // Throws concat error because null.
            SmartDashboard.putString("fail", (e.getMessage() == null) ? "null" : e.getMessage());
          }
          iTemp++;

        } catch (Exception e) {
          // As described above the error message returns null when queried, however, this error is
          // thrown much more frequently. It will occur in bursts every few seconds, so while we are
          // unable to diagnose the issue we leave it in try catch blocks beccause either way we
          // don't want the vision to stop working. It has the same issue with Smart Dashboard as
          // detailed above.
          // System.out.println("loop: " + e.toString()); // Throws concat error because null.
          SmartDashboard.putString("fail", (e.getMessage() == null) ? "null" : e.getMessage());
        }
      }
    });
    somethingThread.setName("vision");
    somethingThread.start();
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
