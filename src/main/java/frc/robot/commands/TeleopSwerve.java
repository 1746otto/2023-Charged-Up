package frc.robot.commands;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private Supplier<Alliance> allianceSupplier;
  private int alliance;
  // private double prevError;

  public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup, Supplier<Alliance> aSupplier) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    allianceSupplier = aSupplier;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    // prevError = 0;

  }

  public boolean joystickBeingUsed(double rotationalVal) {
    if (rotationalVal == 0) {
      return false;
    }
    return true;
  }

  @Override
  public void initialize() {
    if (allianceSupplier.get() == Alliance.Red) {
      alliance = -1;
    } else {
      alliance = 1;
    }
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.stickDeadband);
    // translationVal = Math.copySign(translationVal*translationVal, translationSup.getAsDouble());
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.stickDeadband);
    // translationVal = Math.copySign(translationVal*strafeVal, strafeSup.getAsDouble());
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.stickDeadband);
    Translation2d driveVector = new Translation2d(translationVal, strafeVal);
    driveVector = new Translation2d(Math.pow(driveVector.getNorm(), 1), driveVector.getAngle())
        .times(SwerveConstants.maxSpeed).times(alliance);

    // Slew limiting stuff
    // Translation2d velocityVector = new Translation2d(
    // SwerveConstants.swerveKinematics
    // .toChassisSpeeds(s_Swerve.getModuleStates()).vxMetersPerSecond,
    // SwerveConstants.swerveKinematics
    // .toChassisSpeeds(s_Swerve.getModuleStates()).vyMetersPerSecond);


    // System.out.print("Robot Centric velocity: ");
    // System.out.println(velocityVector);

    // velocityVector.rotateBy(Rotation2d.fromDegrees(s_Swerve.gyro.getYaw()));

    // double changeAngle = Math.atan2(driveVector.getY() - velocityVector.getY(),
    // driveVector.getX() - velocityVector.getX());
    // double magChange = Math.sqrt(
    // (driveVector.getX() - velocityVector.getX()) * (driveVector.getX() - velocityVector.getX())
    // + (driveVector.getY() - velocityVector.getY())
    // * (driveVector.getY() - velocityVector.getY()));
    // System.out.print("Change Angle: ");
    // System.out.println(changeAngle);
    // System.out.print("Change Magnitude: ");
    // System.out.println(magChange);
    // if (magChange > SwerveConstants.slewLimit * slewTimer.get())
    // magChange = SwerveConstants.slewLimit * slewTimer.get();
    // slewTimer.reset();
    // if (Math.abs(velocityVector.getNorm()) < 0.01)
    // driveVector = new Translation2d(velocityVector.getX() + magChange * Math.cos(changeAngle),
    // velocityVector.getY() + magChange * Math.sin(changeAngle));
    // System.out.print("Drive Vector: ");
    // System.out.println(driveVector);
    // System.out.print("Velocity Vector: ");
    // System.out.println(velocityVector);
    /* Drive */
    s_Swerve.drive(driveVector, rotationVal * SwerveConstants.maxAngularVelocity * alliance, true,
        false);
  }
}
