package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionSubsystem.pipelineStates;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class VisionSubsystem extends SubsystemBase {
  private boolean validTarget;
  private double xOffset;
  private double yOffset;
  private double targetArea;
  private double skew;
  private double pipeLatency;
  private double tshort;
  private double tlong;
  private double thor;
  private double tvert;
  private double getpipe;
  private double camtran;
  private int tagID;
  private double[] botPose; //XYZRPY
  private int alliance;
  private String jsonDump;

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
      validTarget = (NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tv")
          .getDouble(0.0) == 1) ? true : false;
      xOffset = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tx")
          .getDouble(0.0);
      yOffset = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("ty")
          .getDouble(0.0);
      targetArea = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("ta")
          .getDouble(0.0);
      skew = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("ts")
          .getDouble(0.0);
      pipeLatency = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tl")
          .getDouble(0.0);
      tshort = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tshort")
          .getDouble(0.0);
      tlong = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tlong")
          .getDouble(0.0);
      thor = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("thor")
          .getDouble(0.0);
      tvert = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tvert")
          .getDouble(0.0);
      getpipe = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("getpipe")
          .getDouble(0.0);
      camtran = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("camtran")
          .getDouble(0.0);
      tagID = (int)NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("tid")
          .getDouble(0.0);
      botPose = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("botpose_wpiblue")
          .getDoubleArray(new double[6]);
      jsonDump = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("json")
          .getString("i don see nuthin");

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

  public double getTargetArea() {
    return targetArea;
  }

  public double getSkew() {
    return skew;
  }

  public double getPipeLatency() {
    return pipeLatency;
  }

  public double getTShort() {
    return tshort;
  }

  public double getTLong() {
    return tlong;
  }

  public double getThor() {
    return thor;
  }

  public double getTvert() {
    return tvert;
  }

  public double getGetPipe() {
    return getpipe;
  }

  public double getCamTran() {
    return camtran;
  }

  public int getTagID() {
    return tagID;
  }

  public String getJSON() {
    return jsonDump;
  }

  public int getNumTags() {
    if(!isTargetValid())
      return 0;
    String json = getJSON();
    int count = 0;
    int place = 0;
    while ((place = json.indexOf("fam", place + 1)) == -1) {
      count++;
    }
    return count;
  }

  public Pose2d getPose2d() {
    return new Pose2d(new Translation2d(botPose[0], botPose[1]), Rotation2d.fromDegrees(botPose[5]));
  }

  public pipelineStates getPipline() {
    switch((int)NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("getpipe").getInteger(0)) {
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
    //     NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("pipeline").setNumber(0);
    //   case RRTAPE:
    //   NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("pipeline").setNumber(1);
    //   default:
    //     return;
    // }
    NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("pipeline").setNumber(0);
  }
  public void setToRRTape(pipelineStates state) {
    NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("pipeline").setNumber(1);
  }

  @Override
  public void periodic() {
      if (alliance == 0) {
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
      fetchvision();
      //System.out.println(getXOffset());
  }
}

