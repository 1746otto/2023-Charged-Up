package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BiConsumer;


public class UpdateOdometryCommand extends CommandBase {
  private VisionSubsystem vision;
  private BiConsumer<Pose2d, Double> poseSetter;
  private double leftLatestTimestamp;
  private double rightLatestTimestamp;

  public UpdateOdometryCommand(VisionSubsystem vision, BiConsumer<Pose2d, Double> poseSetter) {
    this.vision = vision;
    this.poseSetter = poseSetter;
  }

  @Override
  public void execute() {
    if (leftLatestTimestamp != vision.getTimeSinceBootLeft()) {
      leftLatestTimestamp = vision.getTimeSinceBootLeft();
      if (vision.getNumTagsLeft() > 0)
        poseSetter.accept(vision.getPose2dLeft(), vision.getPipeLatencyLeft());
    }
    if (rightLatestTimestamp != vision.getTimeSinceBootRight()) {
      rightLatestTimestamp = vision.getTimeSinceBootRight();
      if (vision.getNumTagsRight() > 0)
        poseSetter.accept(vision.getPose2dRight(), vision.getPipeLatencyRight());
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
