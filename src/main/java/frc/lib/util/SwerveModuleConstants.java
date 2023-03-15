package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final double kS;
  public final double kV;
  public final double kA;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   * 
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
      Rotation2d angleOffset, double kS, double kV, double kA) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
