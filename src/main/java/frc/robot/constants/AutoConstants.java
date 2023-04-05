package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = 2.875;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2.75;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 3;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 1.5;

  public static final double kPDriveController = 0.01;
  public static final double kDDriveController = 0.0;
  public static final double kPThetaController = 0.02;


  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared);
}
