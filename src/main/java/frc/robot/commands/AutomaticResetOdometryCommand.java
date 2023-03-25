


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
  Supplier<Pose2d> currentPoseSupplier;
  Supplier<Pose2d> visionPoseSupplier;
  DoubleSupplier latencySupplier;
  BiConsumer<Pose2d, Double> resetOdometry;
  Timer timer = new Timer();

  public AutomaticResetOdometryCommand(IntSupplier aprilTagSupplier,
      Supplier<Pose2d> currentPoseSupplier, Supplier<Pose2d> visionPoseSupplier,
      DoubleSupplier latencySupplier, BiConsumer<Pose2d, Double> odometrySetter) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.visionPoseSupplier = visionPoseSupplier;
    getNumAprilTags = aprilTagSupplier;
    resetOdometry = odometrySetter;
    this.latencySupplier = latencySupplier;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (getNumAprilTags.getAsInt() >= 1)
      if ((currentPoseSupplier.get().getX() < 3)
          /* blue alliance community zone */ || currentPoseSupplier.get().getX() > 12.5 /* red */)
        if (timer.get() > latencySupplier.getAsDouble())
          resetOdometry.accept(visionPoseSupplier.get(), latencySupplier.getAsDouble());


  }
}
