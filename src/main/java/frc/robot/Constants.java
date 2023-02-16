// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ControllerConstants{
    public static final int kport = 0;
    public static final int kport2 = 1;
    public static final double kdeadzone = .125;
    public static final double kDriveControl = 2.0;

  }
  public static class PlacerConstants {
    public static final int kExtendSolenoidChannel = 12;
    public static final int kRetractSolenoidChannel = 13;
    public static final int kChannel = 0;
    public static final boolean kPlacerEngaged = false;
  }
  public static class RobotConstants{
    public static final int kREVPH = 2;
  }
  public static class IntakeRollerConstants {
    public static final int CANID1= 10;
    public static final int CANID2= 11;
    public static final double kFullPower = 0.4;
  }

  public static class IntakeExtendConstants {
    public static final int CANID1 = 10;
    public static final int CANID2 = 11;
    public static final double kFullPower = 0.4;
    public static final int kLimitSwitch1 = 1;
    public static final int kLimitSwitch2 = 1;
    public static final double kZeroPower = 0.0;
  }
}