package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

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

    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 5;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final String CANBus = "CANivore"; //optional, only required if using a CANivore.

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
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

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.01; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 2; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 1 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.687500+180);//87.187500
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(247.236328);//108.808594
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */ 
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.732422+180);//328.623047
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(85.341797+180);//158.378906
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
      public static class PlungerConstants {
        public static final int kExtendSolenoidChannel = 12;
        public static final int kRetractSolenoidChannel = 13;
        public static final int kChannel = 0;
        public static final boolean kPlacerEngaged = false;
      }

    public static class IndexerConstants{
      public static final int kIndexerMotor = 31;
      public static final int kIndexerMotor2 = 32;
      public static final int kIndexerMotor3 = 33;
    public static int speed = 2;
    public static int reverseSpeed = -2;
    public static final int kChannel = 2;
    public static final int kExtendSolenoidChannel = 12;
    public static final int kRetractSolenoidChannel = 13;
   public static final int kbeambreak = 3;
  }
  public static class ClamperConstants {
    public static final int kExtendSolenoidChannel = 14;
    public static final int kRetractSolenoidChannel = 15;
    public static final int kChannel = 0;
    public static final boolean kPlacerEngaged = false;
  }
  public static class ElevatorConstants{
    public static final int kElevatorMotor1ID = 41;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorP = 1.0;
    public static final int kOriginPosition = 0;
    public static final int kMidPosition = 48;
    public static final int kHighPosition = 55;
    public static final double kElevatorSpeed = 1.0;
    // TODO: change channel num to proper num
    public static final int kElevatorAnalogInputChannel = 0;
  }
  public static class FlapConstants{
    // TODO: change flap channel num to proper num
    public static final int kFlapChannelID = 0;
  }
}