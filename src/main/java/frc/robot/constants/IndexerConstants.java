package frc.robot.constants;

public final class IndexerConstants {
  // CAN IDs
  public static final int kIndexerTreadMotor = 31;
  public static final int kIndexerRightRollerMotor = 32;
  public static final int kIndexerLeftRollerMotor = 33;

  // RIO Analog IO Ports
  public static final int kbeamBreak = 1;

  // Tunable constants
  public static final double kRollerIntakeSpeed = 1;
  public static final double kRollerOuttakeSpeed = -.4;
  public static final double kRollerStopSpeed = 0;

  public static final double kTreadIntakeSpeed = .5;
  public static final double kTreadOuttakeSpeed = -.7;
  public static final double kTreadScoreSpeed = 1;
  public static final double kTreadStopSpeed = 0;

  public static final double kBeamBreakBrokenVoltage = .67;
}
