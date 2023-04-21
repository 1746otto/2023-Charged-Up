package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class UpdateOdometryCommand extends CommandBase {
  private VisionSubsystem vision;
  private DoubleSupplier gyroAngle;
  private Supplier<Pose2d> currentPose;
  private BiConsumer<Pose2d, Double> poseSetter;
  private double leftLatestTimestamp;
  private double rightLatestTimestamp;
  private double halfFieldLength = 8.270367;
  private double fieldWidth = 8.02;

  public UpdateOdometryCommand(VisionSubsystem vision, DoubleSupplier gyroAngle,
      Supplier<Pose2d> currentPose, BiConsumer<Pose2d, Double> poseSetter) {
    this.vision = vision;
    this.gyroAngle = gyroAngle;
    this.currentPose = currentPose;
    this.poseSetter = poseSetter;
    addRequirements(this.vision);

    leftLatestTimestamp = 0;
    rightLatestTimestamp = 0;
  }

  @Override
  public void execute() {
    if (vision.getTimeSinceBootLeft() > 0.5 && vision.getTimeSinceBootRight() > 0.5) {
      if (DriverStation.isAutonomous()) {
        if (DriverStation.getAlliance() == Alliance.Red) {
          if (leftLatestTimestamp != vision.getTimeSinceBootLeft()) {
            leftLatestTimestamp = vision.getTimeSinceBootLeft();
            if (vision.getNumTagsLeft() > 0) {
              Pose2d newPose = new Pose2d(
                  new Translation2d(
                      -(vision.getPose2dLeft().getX() - halfFieldLength) + halfFieldLength,
                      fieldWidth - vision.getPose2dLeft().getY()),
                  Rotation2d.fromDegrees(gyroAngle.getAsDouble()));
              if (newPose.getTranslation().getDistance(currentPose.get().getTranslation()) > 1) {
                poseSetter.accept(newPose, Timer.getFPGATimestamp() - vision.getPipeLatencyLeft()
                    - vision.getCaptureLatencyLeft());
              }
            }
          }
          if (rightLatestTimestamp != vision.getTimeSinceBootRight()) {
            rightLatestTimestamp = vision.getTimeSinceBootRight();
            if (vision.getNumTagsRight() > 0) {
              Pose2d newPose = new Pose2d(
                  new Translation2d(
                      -(vision.getPose2dRight().getX() - halfFieldLength) + halfFieldLength,
                      fieldWidth - vision.getPose2dRight().getY()),
                  Rotation2d.fromDegrees(gyroAngle.getAsDouble()));
              if (newPose.getTranslation().getDistance(currentPose.get().getTranslation()) > 1) {
                poseSetter.accept(newPose, Timer.getFPGATimestamp() - vision.getPipeLatencyRight()
                    - vision.getCaptureLatencyRight());
              }
            }
          }
        }
      } else {
        if (leftLatestTimestamp != vision.getTimeSinceBootLeft()) {
          leftLatestTimestamp = vision.getTimeSinceBootLeft();
          if (vision.getNumTagsLeft() > 0) {
            Pose2d newPose = new Pose2d(vision.getPose2dLeft().getTranslation(),
                Rotation2d.fromDegrees(gyroAngle.getAsDouble()));
            if (newPose.getTranslation().getDistance(currentPose.get().getTranslation()) > 1) {
              poseSetter.accept(newPose, Timer.getFPGATimestamp() - vision.getPipeLatencyLeft()
                  - vision.getCaptureLatencyLeft());
            }
          }
        }
        if (rightLatestTimestamp != vision.getTimeSinceBootRight()) {
          rightLatestTimestamp = vision.getTimeSinceBootRight();
          if (vision.getNumTagsRight() > 0) {
            Pose2d newPose = new Pose2d(vision.getPose2dRight().getTranslation(),
                Rotation2d.fromDegrees(gyroAngle.getAsDouble()));
            if (newPose.getTranslation().getDistance(currentPose.get().getTranslation()) > 1) {
              poseSetter.accept(newPose, Timer.getFPGATimestamp() - vision.getPipeLatencyRight()
                  - vision.getCaptureLatencyRight());
            }
          }
        }
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
