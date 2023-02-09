package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private double[] botPose; //XYZRPY

  public VisionSubsystem() {

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
      botPose = NetworkTableInstance.getDefault().getTable("limelight-otto").getEntry("botpose").getDoubleArray(new double[6]);
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

  public Pose2d getPose2d() {
    return new Pose2d(new Translation2d(botPose[0], botPose[2]), Rotation2d.fromDegrees(botPose[5]));
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      fetchvision();
      System.out.println(getPose2d().getRotation());
  }
}

