package frc.robot.constants;

public final class ArmConstants {
  // TODO: change values to accurate positions and values
  // TODO: chnage positions to match cancoder.
  public static final double kArmRestPos = 0.0;
  public static final double kArmIntakeAndScorePos = -18640.0;
  // TODO: change cube intake pos
  // TODO: change positions constants to match cancoder.
  public static final double kArmConeIntakePos = -18640.0;
  public static final double kArmHighScoringPos = -14776.0;
  public static final double kArmP = .4;
  public static final double kRollerSpeed = 0.5;
  public static final double kRollerStowSpeed = 0.08;
  public static final double kArmCurrentMax = 70; // 70.0;
  public static final double kArmGearRatio = 3; // 3:1 Previously 9.0
  public static final double kCANTickToFalConversion = .5;
  public static final int kArmPosMotorID = 51;
  public static final int kArmRollerMotorID = 31;
  public static final int kCANCoderID = 50;
}
