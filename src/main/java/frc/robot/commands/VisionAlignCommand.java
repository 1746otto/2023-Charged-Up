package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends CommandBase {
  int stage;
  Pose2d pivotPose;
  VisionSubsystem vision;
  Swerve swerve;
  DoubleSupplier rotateSupplier;
  DoubleSupplier forwardSupplier;

  public VisionAlignCommand(VisionSubsystem vision, Swerve swerve, DoubleSupplier rotateSupplier,
      DoubleSupplier forwardSupplier) {
    this.vision = vision;
    this.swerve = swerve;
    this.rotateSupplier = rotateSupplier;
    this.forwardSupplier = forwardSupplier;
  }

  @Override
  public void initialize() {
    // vision.
  }
}
