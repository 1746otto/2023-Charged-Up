package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutomaticResetOdometryCommand extends CommandBase {

  IntSupplier getNumAprilTags;
  Supplier<Pose2d> poseSupplier;
  DoubleSupplier latencySupplier;
  BiConsumer<Pose2d, Double> resetOdometry;
  Timer timer;

  public AutomaticResetOdometryCommand(IntSupplier aprilTagSupplier, Supplier<Pose2d> poseSupplier,
      DoubleSupplier latencySupplier, BiConsumer<Pose2d, Double> odometrySetter) {
    this.poseSupplier = poseSupplier;
    getNumAprilTags = aprilTagSupplier;
    resetOdometry = odometrySetter;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (getNumAprilTags.getAsInt() >= 3)
      if ((poseSupplier.get().getX() < 3)
          /* blue alliance community zone */ || poseSupplier.get().getX() > 12.5 /* red */)
        if (timer.get() > latencySupplier.getAsDouble())
          resetOdometry.accept(poseSupplier.get(), latencySupplier.getAsDouble());

  }
}
