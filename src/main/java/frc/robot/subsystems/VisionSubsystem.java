package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.LimeLightHelpers.LimelightResults;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final String kLimeLightNameLeft = "limelight-lotto";
  private final String kLimeLightNameRight = "limelight-rotto";

  private boolean validTargetLeft;
  private double xOffsetLeft;
  private double yOffsetLeft;
  private double pipeLatencyLeft;
  private double captureLatencyLeft;
  private double getpipeLeft;
  private int tagIDLeft;

  private boolean validTargetRight;
  private double xOffsetRight;
  private double yOffsetRight;
  private double pipeLatencyRight;
  private double captureLatencyRight;
  private double getpipeRight;
  private int tagIDRight;
  // private double[] botPose; // XYZRPY
  private int alliance;
  private LimelightResults llResultsLeft;
  private LimelightResults llResultsRight;

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
      while (true) {
        fetchvision();
        Timer.delay(0.04);
      }
    });
  }

  public void fetchvision() {
    try {
      llResultsLeft = LimeLightHelpers.getLatestResults(kLimeLightNameLeft);
      validTargetLeft = llResultsLeft.targetingResults.valid;
      xOffsetLeft = NetworkTableInstance.getDefault().getTable(kLimeLightNameLeft).getEntry("tx")
          .getDouble(0.0);
      yOffsetLeft = NetworkTableInstance.getDefault().getTable(kLimeLightNameLeft).getEntry("ty")
          .getDouble(0.0);
      pipeLatencyLeft = llResultsLeft.targetingResults.latency_pipeline;
      captureLatencyLeft = llResultsLeft.targetingResults.latency_capture;
      getpipeLeft = llResultsLeft.targetingResults.pipelineID;
      tagIDLeft = (int) NetworkTableInstance.getDefault().getTable(kLimeLightNameLeft)
          .getEntry("tid").getInteger(0);

      llResultsRight = LimeLightHelpers.getLatestResults(kLimeLightNameRight);
      validTargetRight = llResultsRight.targetingResults.valid;
      xOffsetRight = NetworkTableInstance.getDefault().getTable(kLimeLightNameRight).getEntry("tx")
          .getDouble(0.0);
      yOffsetRight = NetworkTableInstance.getDefault().getTable(kLimeLightNameRight).getEntry("ty")
          .getDouble(0.0);
      pipeLatencyRight = llResultsRight.targetingResults.latency_pipeline;
      captureLatencyRight = llResultsRight.targetingResults.latency_capture;
      getpipeRight = llResultsRight.targetingResults.pipelineID;
      tagIDRight = (int) NetworkTableInstance.getDefault().getTable(kLimeLightNameRight)
          .getEntry("tid").getInteger(0);
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
  }

  public boolean isTargetValidLeft() {
    return validTargetLeft;
  }

  public double getXOffsetLeft() {
    return xOffsetLeft;
  }

  public double getYOffsetLeft() {
    return yOffsetLeft;
  }

  public double getPipeLatencyLeft() {
    return pipeLatencyLeft;
  }

  public double getCaptureLatencyLeft() {
    return captureLatencyLeft;
  }

  public double getGetPipeLeft() {
    return getpipeLeft;
  }

  public int getTagIDLeft() {
    return tagIDLeft;
  }

  public int getNumTagsLeft() {
    return llResultsLeft.targetingResults.targets_Fiducials.length;
  }

  public Pose2d getPose2dLeft() {
    return llResultsLeft.targetingResults.getBotPose2d_wpiBlue();
  }

  public boolean isTargetValidRight() {
    return validTargetRight;
  }

  public double getXOffsetRight() {
    return xOffsetRight;
  }

  public double getYOffsetRight() {
    return yOffsetRight;
  }

  public double getPipeLatencyRight() {
    return pipeLatencyRight;
  }

  public double getCaptureLatencyRight() {
    return captureLatencyRight;
  }

  public double getGetPipeRight() {
    return getpipeRight;
  }

  public int getTagIDRight() {
    return tagIDRight;
  }

  public int getNumTagsRight() {
    return llResultsRight.targetingResults.targets_Fiducials.length;
  }

  public Pose2d getPose2dRight() {
    return llResultsRight.targetingResults.getBotPose2d_wpiBlue();
  }

  public double getTimeSinceBootLeft() {
    return llResultsLeft.targetingResults.timestamp_LIMELIGHT_publish;
  }

  public double getTimeSinceBootRight() {
    return llResultsRight.targetingResults.timestamp_LIMELIGHT_publish;
  }

  public int getAlliance() {
    return alliance;
  }

  public pipelineStates getPiplineLeft() {
    switch ((int) getpipeLeft) {
      case 0:
        return pipelineStates.APRILTAG;
      case 1:
        return pipelineStates.RRTAPE;
      default:
        return pipelineStates.ERROR;
    }
  }

  public pipelineStates getPiplineRight() {
    switch ((int) getpipeRight) {
      case 0:
        return pipelineStates.APRILTAG;
      case 1:
        return pipelineStates.RRTAPE;
      default:
        return pipelineStates.ERROR;
    }
  }

  public void setLeftState(pipelineStates state) {
    switch (state) {
      case APRILTAG:
        NetworkTableInstance.getDefault().getTable(kLimeLightNameLeft).getEntry("pipeline")
            .setNumber(VisionConstants.kPipelineApriltag);
      case RRTAPE:
        NetworkTableInstance.getDefault().getTable(kLimeLightNameLeft).getEntry("pipeline")
            .setNumber(VisionConstants.kPipleineRetroTape);
      case ERROR:
        return;
    }
  }

  public void setRightState(pipelineStates state) {
    switch (state) {
      case APRILTAG:
        NetworkTableInstance.getDefault().getTable(kLimeLightNameRight).getEntry("pipeline")
            .setNumber(VisionConstants.kPipelineApriltag);
      case RRTAPE:
        NetworkTableInstance.getDefault().getTable(kLimeLightNameRight).getEntry("pipeline")
            .setNumber(VisionConstants.kPipleineRetroTape);
      case ERROR:
        return;
    }
  }

  @Override
  public void periodic() {
    // fetchvision();
  }
}
