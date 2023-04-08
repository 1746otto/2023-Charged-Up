package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class SwerveConstants {
  public static final int pigeonID = 5;
  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  public static final String CANBus = "CANivore"; // optional, only required if using a CANivore.

  public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to
                                                               // specific robot
      COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

  /* Drivetrain Constants */
  public static final double trackWidth = Units.inchesToMeters(18.75); // TODO: This must be tuned
                                                                       // to specific robot
  public static final double wheelBase = Units.inchesToMeters(18.75); // TODO: This must be tuned to
                                                                      // specific robot
  public static final double wheelCircumference = chosenModule.wheelCircumference;
  public static final double slewLimit = 1.5; // Max acceleration meters per second squared.

  /*
   * Swerve Kinematics No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  public static final BetterSwerveKinematics betterSwerveKinematics =
      new BetterSwerveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  /* Module Gear Ratios */
  public static final double driveGearRatio = chosenModule.driveGearRatio;
  public static final double angleGearRatio = chosenModule.angleGearRatio;

  /* Motor Inverts */
  public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
  public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = chosenModule.canCoderInvert;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 25;
  public static final int anglePeakCurrentLimit = 40;
  public static final double anglePeakCurrentDuration = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  public static final int driveContinuousCurrentLimit = 35;
  public static final int drivePeakCurrentLimit = 60;
  public static final double drivePeakCurrentDuration = 0.1;
  public static final boolean driveEnableCurrentLimit = true;

  /*
   * These values are used by the drive falcon to ramp in open loop and closed loop driving. We
   * found a small open loop ramp (0.25) helps with tread wear, tipping, etc
   */
  public static final double openLoopRamp = 0.0;
  public static final double closedLoopRamp = 0.0;
  public static final double openLoopRampAngle = 0.15;

  /* Angle Motor PID Values */
  public static final double angleKP = chosenModule.angleKP;
  public static final double angleKI = chosenModule.angleKI;
  public static final double angleKD = chosenModule.angleKD;
  public static final double angleKF = chosenModule.angleKF;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.01; // TODO: This must be tuned to specific robot
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKF = 0.0;

  /*
   * Drive Motor Characterization Values Divide SYSID values by 12 to convert from volts to percent
   * output for CTRE
   */
  public static final double driveKS = (0.22528 / 12.0); // TODO: This must be tuned cuz its bad
  public static final double driveKV = (1.8397 / 12.0);
  public static final double driveKA = (0.19052 / 12.0);

  /* Swerve Profiling Values */
  /** Meters per Second */
  public static double maxSpeed = 13.5; // TODO: This must be tuned to specific robot
  /** Radians per Second */
  public static double maxAngularVelocity = 10; // TODO: This must be tuned to specific robot

  public static final double autonDriveSpeed = 0.250;

  /* Neutral Modes */
  public static NeutralMode angleNeutralMode = NeutralMode.Coast;
  public static NeutralMode driveNeutralMode = NeutralMode.Coast;

  /* Module Specific Constants */
  /* Front Left Module - Module 0 */
  public static final class Mod0 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 11;
    public static final int angleMotorID = 12;
    public static final int canCoderID = 10;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.335938 - 180);// 87.187500
    public static final double kS = driveKS;
    public static final double kV = driveKV;
    public static final double kA = driveKA;

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, kS, kV, kA);
  }

  /* Front Right Module - Module 1 */
  public static final class Mod1 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 31;
    public static final int angleMotorID = 32;
    public static final int canCoderID = 30;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(247.587891);// 108.808594
    public static final double kS = driveKS;
    public static final double kV = driveKV;
    public static final double kA = driveKA;

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, kS, kV, kA);
  }

  /* Back Left Module - Module 2 */
  public static final class Mod2 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 1;
    public static final int angleMotorID = 2;
    public static final int canCoderID = 0;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(151.699219);// 328.623047
    public static final double kS = driveKS;
    public static final double kV = driveKV;
    public static final double kA = driveKA;


    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, kS, kV, kA);
  }

  /* Back Right Module - Module 3 */
  public static final class Mod3 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 21;
    public static final int angleMotorID = 22;
    public static final int canCoderID = 20;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(30.410156 + 180);// 158.378906
    public static final double kS = driveKS;
    public static final double kV = driveKV;
    public static final double kA = driveKA;


    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, kS, kV, kA);
  }
}
