package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.BetterSwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  public SwerveModule[] mSwerveMods;
  public WPI_Pigeon2 gyro;
  public final SwerveDrivePoseEstimator poseEstimator;
  private boolean spamRestart = false;


  public Swerve() {
    gyro = new WPI_Pigeon2(SwerveConstants.pigeonID, (SwerveConstants.CANBus));
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods = new SwerveModule[] {new SwerveModule(0, SwerveConstants.Mod0.constants),
        new SwerveModule(1, SwerveConstants.Mod1.constants),
        new SwerveModule(2, SwerveConstants.Mod2.constants),
        new SwerveModule(3, SwerveConstants.Mod3.constants)};

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug with inverting
     * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    // for (int i = 0; i < 10; i++) {
    // Timer.delay(.2);
    // resetModulesToAbsolute();
    // for (SwerveModule mod : mSwerveMods) {
    // if (mod.angleEncoder.getLastError() != ErrorCode.OK)
    // spamRestart = true;
    // }
    // }


    poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(),
        getModulePositions(), new Pose2d());
  }

  public void setDriveNeutralMode(NeutralMode mode) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setModuleNeutralMode(mode);
    }
  }

  public double getMagnitude(Translation2d translation) {
    return Math
        .sqrt(translation.getX() * translation.getX() + translation.getY() * translation.getY());
  }

  // fieldRelative switches the speeds from fieldRelative to robotRelative and vice versa
  public void drive(Translation2d translation, double rotation, boolean fieldRelative,
      boolean isOpenLoop) {
    BetterSwerveModuleState[] swerveModuleStates =
        SwerveConstants.betterSwerveKinematics.toSwerveModuleStates(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, getYaw())
            : new ChassisSpeeds((translation.getX()), (translation.getY()), rotation));
    BetterSwerveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(BetterSwerveModuleState[] desiredStates) {
    BetterSwerveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public BetterSwerveModuleState[] getModuleStates() {
    BetterSwerveModuleState[] states = new BetterSwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getBetterModuleState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(180);
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
      if (mod.angleEncoder.getLastError() != ErrorCode.OK)
        spamRestart = true;
    }
  }

  public void XLock() {
    mSwerveMods[0].setAngleNoDeadzone(Rotation2d.fromDegrees(45));
    mSwerveMods[1].setAngleNoDeadzone(Rotation2d.fromDegrees(135));
    mSwerveMods[2].setAngleNoDeadzone(Rotation2d.fromDegrees(135));
    mSwerveMods[3].setAngleNoDeadzone(Rotation2d.fromDegrees(45));
  }

  public void addVisionMeasurement(Pose2d pose, double latency) {
    poseEstimator.addVisionMeasurement(pose, latency);
  }

  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getModulePositions());
    if (poseEstimator != null) {
      SmartDashboard.putNumber("Positionx", getPose().getX());
      SmartDashboard.putNumber("Positiony", getPose().getY());
      SmartDashboard.putNumber("Positiontheta", getPose().getRotation().getDegrees());
    }
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
          mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
          mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
          mod.getPosition().distanceMeters);

    }
    if (spamRestart)
      System.out.println("Restart the robot or it will suck!");
  }
}
