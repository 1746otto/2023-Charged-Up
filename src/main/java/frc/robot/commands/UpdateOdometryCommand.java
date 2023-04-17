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


public class UpdateOdometryCommand extends CommandBase {
  private VisionSubsystem vision;
  private BiConsumer<Pose2d, Double> poseSetter;
  private double leftLatestTimestamp;
  private double rightLatestTimestamp;
  private double halfFieldLength = 8.270367;
  private double fieldWidth = 8.02;

  public UpdateOdometryCommand(VisionSubsystem vision, BiConsumer<Pose2d, Double> poseSetter) {
    this.vision = vision;
    this.poseSetter = poseSetter;
    addRequirements(this.vision);
  }

  @Override
  public void execute() {
    if (DriverStation.isAutonomous()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        if (leftLatestTimestamp != vision.getTimeSinceBootLeft()) {
          leftLatestTimestamp = vision.getTimeSinceBootLeft();
          if (vision.getNumTagsLeft() > 0)
            poseSetter.accept(
                new Pose2d(new Translation2d(
                    -(vision.getPose2dLeft().getX() - halfFieldLength) + halfFieldLength,
                    fieldWidth - vision.getPose2dLeft().getY()),
                    vision.getPose2dLeft().getRotation()
                        .plus(Rotation2d.fromDegrees(180).times(-1))),
                Timer.getFPGATimestamp() - vision.getPipeLatencyLeft()
                    - vision.getCaptureLatencyLeft());
        }
        if (rightLatestTimestamp != vision.getTimeSinceBootRight()) {
          rightLatestTimestamp = vision.getTimeSinceBootRight();
          if (vision.getNumTagsRight() > 0)
            poseSetter.accept(
                new Pose2d(new Translation2d(
                    -(vision.getPose2dRight().getX() - halfFieldLength) + halfFieldLength,
                    fieldWidth - vision.getPose2dRight().getY()),
                    vision.getPose2dRight().getRotation()
                        .plus(Rotation2d.fromDegrees(180).times(-1))),
                Timer.getFPGATimestamp() - vision.getPipeLatencyRight()
                    - vision.getCaptureLatencyRight());
        }
      }
    } else {
      if (leftLatestTimestamp != vision.getTimeSinceBootLeft()) {
        leftLatestTimestamp = vision.getTimeSinceBootLeft();
        if (vision.getNumTagsLeft() > 0)
          poseSetter.accept(vision.getPose2dLeft(), Timer.getFPGATimestamp()
              - vision.getPipeLatencyLeft() - vision.getCaptureLatencyLeft());
      }
      if (rightLatestTimestamp != vision.getTimeSinceBootRight()) {
        rightLatestTimestamp = vision.getTimeSinceBootRight();
        if (vision.getNumTagsRight() > 0)
          poseSetter.accept(vision.getPose2dRight(), Timer.getFPGATimestamp()
              - vision.getPipeLatencyRight() - vision.getCaptureLatencyRight());
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
