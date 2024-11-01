package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.SwerveConstants;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  public CANCoder angleEncoder;

  private double CANCoderInitTime = 0.0;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
      SwerveConstants.driveKV, SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, SwerveConstants.CANBus);
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID, SwerveConstants.CANBus);
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID, SwerveConstants.CANBus);
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setModuleNeutralMode(NeutralMode mode) {
    mDriveMotor.setNeutralMode(mode);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes continuous
     * controller which CTRE and Rev onboard is not
     */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
      mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
          SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
      mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }



  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents
                                  // Jittering.

    mAngleMotor.set(ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio));
    lastAngle = angle;
  }

  public void setAngleNoDeadzone(Rotation2d angle) {
    mAngleMotor.set(ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio));
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Conversions
        .falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
  }

  // Maybe change this to be the normal position so it will be the same thing.
  // public Rotation2d getCanCoder() {
  // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  // }
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    waitForCANCoder();
    double absolutePosition = Conversions.degreesToFalcon(
        getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    mAngleMotor.configFactoryDefault();
    mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    mAngleMotor.setInverted(SwerveConstants.angleMotorInvert);
    mAngleMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    mDriveMotor.configFactoryDefault();
    mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.setInverted(SwerveConstants.driveMotorInvert);
    mDriveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio), getAngle());
  }

  public void waitForCANCoder() {
    double firstGoodTimestamp = 0;
    for (int i = 0; i < 100; i++) {
      angleEncoder.getAbsolutePosition();
      if (angleEncoder.getLastError() == ErrorCode.OK) {
        firstGoodTimestamp = angleEncoder.getLastTimestamp();
        break;
      }
      Timer.delay(0.010);
      CANCoderInitTime += 10;
    }
    for (int i = 0; i < 100; i++) {
      angleEncoder.getAbsolutePosition();
      if (angleEncoder.getLastError() == ErrorCode.OK
          && angleEncoder.getLastTimestamp() != firstGoodTimestamp) {
        break;
      }
      Timer.delay(0.010);
      CANCoderInitTime += 10;
    }
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
            SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
        getAngle());
  }
}
