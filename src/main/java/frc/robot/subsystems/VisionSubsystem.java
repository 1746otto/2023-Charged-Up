package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.LimeLightHelpers.LimelightResults;
import frc.robot.LimeLightHelpers.Results;
import frc.robot.subsystems.VisionSubsystem.pipelineStates;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class VisionSubsystem extends SubsystemBase {
  private boolean validTarget;
  private double xOffset;
  private double yOffset;
  private double pipeLatency;
  private double captureLatency;
  private double getpipe;
  private int tagID;
  private double[] botPose; //XYZRPY
  private int alliance;
  private LimelightResults llResults;
  private final String limeLightName = "limelight-otto";

  public static enum pipelineStates{APRILTAG, RRTAPE, ERROR};

  public VisionSubsystem() {
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      alliance = -1;
    }
    else if (DriverStation.getAlliance().equals(Alliance.Red)) {
      alliance = 1;
    }
    else {
      alliance = 0;
    }
  }

  public void fetchvision() {
    try {
      llResults = LimeLightHelpers.getLatestResults(limeLightName);
      validTarget = llResults.targetingResults.valid;
      xOffset = NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("tx")
          .getDouble(0.0);
      yOffset = NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("ty")
          .getDouble(0.0);
      pipeLatency = llResults.targetingResults.latency_pipeline;
      captureLatency = llResults.targetingResults.latency_capture;
      getpipe = llResults.targetingResults.pipelineID;
      tagID = (int)NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("tid")
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

  public pipelineStates getPipline() {
    switch((int)getpipe) {
      case 0:
        return pipelineStates.APRILTAG;
      case 1:
        return pipelineStates.RRTAPE;
      default:
        return pipelineStates.ERROR;
    }
  }

  public void setToAprilTag(pipelineStates state) {
    // switch(state) {
    //   case APRILTAG:
    //     NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("pipeline").setNumber(0);
    //   case RRTAPE:
    //   NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("pipeline").setNumber(1);
    //   default:
    //     return;    
    // }
    
    NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("pipeline").setNumber(0);
  }
  public void setToRRTape(pipelineStates state) {
    NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("pipeline").setNumber(1);
  }

  @Override
  public void periodic() {
      try {
      System.out.println(getNumTags());
      } catch (Exception e) {

      }
      fetchvision();
  }
}

