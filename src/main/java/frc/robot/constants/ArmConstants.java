package frc.robot.constants;

public final class ArmConstants {
  // TODO: change values to accurate positions and values
  // TODO: chnage positions to match cancoder.
  public static final double kArmRestPos = -.02; // 0.5
  public static final double kArmIntakeAndScorePos = -6.3383; // CANCoder: 0.9065 human player pos:
                                                              // 0.5634
  // TODO: change cube intake pos
  // TODO: change positions constants to match cancoder.
  public static final double kArmConeIntakePos = -2.7741;// Tal:-2.7741; // CAN:0.7521
  public static final double kArmHighScoringPos = -4.543;
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
