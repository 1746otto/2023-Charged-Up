package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.LimeLightHelpers.LimelightResults;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final String kLimeLightName = "limelight-otto";

  private boolean validTarget;
  private double xOffset;
  private double yOffset;
  private double pipeLatency;
  private double captureLatency;
  private double getpipe;
  private int tagID;
  // private double[] botPose; // XYZRPY
  private int alliance;
  private LimelightResults llResults;

  public static enum pipelineStates {
    APRILTAG, RRTAPE, ERROR
  };

  public VisionSubsystem() {
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      alliance = -1;
    } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
      alliance = 1;
    } else {
      alliance = 0;
    }
    new Thread(() -> {
      fetchvision();
    });
  }

  public void fetchvision() {
    try {
      llResults = LimeLightHelpers.getLatestResults(kLimeLightName);
      validTarget = llResults.targetingResults.valid;
      xOffset =
          NetworkTableInstance.getDefault().getTable(kLimeLightName).getEntry("tx").getDouble(0.0);
      yOffset =
          NetworkTableInstance.getDefault().getTable(kLimeLightName).getEntry("ty").getDouble(0.0);
      pipeLatency = llResults.targetingResults.latency_pipeline;
      captureLatency = llResults.targetingResults.latency_capture;
      getpipe = llResults.targetingResults.pipelineID;
      tagID = (int) NetworkTableInstance.getDefault().getTable(kLimeLightName).getEntry("tid")
          .getDouble(0.0);
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
  }

  public boolean isTargetValid() {
    return validTarget;
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getPipeLatency() {
    return pipeLatency;
  }

  public double getCaptureLatency() {
    return captureLatency;
  }

  public double getGetPipe() {
    return getpipe;
  }

  public int getTagID() {
    return tagID;
  }

  public int getNumTags() {
    return llResults.targetingResults.targets_Fiducials.length;
  }

  public Pose2d getPose2d() {
    return llResults.targetingResults.getBotPose2d_wpiBlue();
  }

  public int getAlliance() {
    return alliance;
  }

  public pipelineStates getPipline() {
    switch ((int) getpipe) {
      case 0:
        return pipelineStates.APRILTAG;
      case 1:
        return pipelineStates.RRTAPE;
      default:
        return pipelineStates.ERROR;
    }
  }

  public void setToAprilTag(pipelineStates state) {
    NetworkTableInstance.getDefault().getTable(kLimeLightName).getEntry("pipeline")
        .setNumber(VisionConstants.kPipelineApriltag);
  }

  public void setToRRTape(pipelineStates state) {
    NetworkTableInstance.getDefault().getTable(kLimeLightName).getEntry("pipeline")
        .setNumber(VisionConstants.kPipleineRetroTape);
  }

  @Override
  public void periodic() {
    // fetchvision();
  }
}

