package frc.robot.commands;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
  private double kP = 0.015;
  // private double kD = 1;

  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier faceUpSup;
  private BooleanSupplier faceDownSup;
  private BooleanSupplier faceRightSup;
  private BooleanSupplier faceLeftSup;
  Timer slewTimer;
  private int rotationAngle;
  // private double prevError;

  public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier faceUpSup,
      BooleanSupplier faceDownSup, BooleanSupplier faceRightSup, BooleanSupplier faceLeftSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.faceUpSup = faceUpSup;
    this.faceDownSup = faceDownSup;
    this.faceLeftSup = faceLeftSup;
    this.faceRightSup = faceRightSup;
    rotationAngle = -600;
    slewTimer = new Timer();
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
    slewTimer.start();
  }

  @Override
  public void execute() {
    // Turning Buttons
    if (faceUpSup.getAsBoolean()) {
      rotationAngle = 0;
    }
    if (faceDownSup.getAsBoolean()) {
      rotationAngle = 180;
    }
    if (faceRightSup.getAsBoolean()) {
      rotationAngle = 270;
    }
    if (faceLeftSup.getAsBoolean()) {
      rotationAngle = 90;
    }
    /* Get Values, Deadband */
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.stickDeadband);
    // translationVal = Math.copySign(translationVal*translationVal, translationSup.getAsDouble());
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.stickDeadband);
    // translationVal = Math.copySign(translationVal*strafeVal, strafeSup.getAsDouble());
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.stickDeadband);
    Translation2d driveVector = new Translation2d(translationVal, strafeVal)
        .times(Math.sqrt(translationVal * translationVal + strafeVal * strafeVal))
        .times(SwerveConstants.maxSpeed);
    double currentAngle = MathUtil.inputModulus(s_Swerve.getYaw().getDegrees(), 0, 360);

    if (joystickBeingUsed(rotationVal)) {
      rotationAngle = -600;
      // prevError = 0;
    }

    if (rotationAngle >= 0) {
      double clockwiseDist = MathUtil.inputModulus(currentAngle - rotationAngle, 0, 360);
      double counterClockwiseDist = 360 - clockwiseDist;// 360 - MathUtil.inputModulus(currentAngle,
                                                        // 0, 360);

      boolean moveCounterClockwise = !(clockwiseDist <= counterClockwiseDist);

      // Set initial rotation velocity at max in desired direction of travel
      double error;
      if (moveCounterClockwise) {
        error = counterClockwiseDist;
        rotationVal = .8;
      } else {
        error = -clockwiseDist;
        rotationVal = -.8;
      }

      rotationVal = kP * error;
      // rotationVal = (error - prevError) * kD + (kP * error);

      if (rotationVal > 1) {
        rotationVal = 1;
      } else if (rotationVal < -1) {
        rotationVal = -1;
      }

      // prevError = error;
    }


    // Slew limiting stuff
    Translation2d velocityVector = new Translation2d(
        SwerveConstants.swerveKinematics
            .toChassisSpeeds(s_Swerve.getModuleStates()).vxMetersPerSecond,
        SwerveConstants.swerveKinematics
            .toChassisSpeeds(s_Swerve.getModuleStates()).vyMetersPerSecond);


    System.out.print("Robot Centric velocity: ");
    System.out.println(velocityVector);

    velocityVector.rotateBy(Rotation2d.fromDegrees(s_Swerve.gyro.getYaw()));

    double changeAngle = Math.atan2(driveVector.getY() - velocityVector.getY(),
        driveVector.getX() - velocityVector.getX());
    double magChange = Math.sqrt(
        (driveVector.getX() - velocityVector.getX()) * (driveVector.getX() - velocityVector.getX())
            + (driveVector.getY() - velocityVector.getY())
                * (driveVector.getY() - velocityVector.getY()));
    System.out.print("Change Angle: ");
    System.out.println(changeAngle);
    System.out.print("Change Magnitude: ");
    System.out.println(magChange);
    if (magChange > SwerveConstants.slewLimit * slewTimer.get())
      magChange = SwerveConstants.slewLimit * slewTimer.get();
    slewTimer.reset();
    if (Math.abs(velocityVector.getNorm()) < 0.01)
      driveVector = new Translation2d(velocityVector.getX() + magChange * Math.cos(changeAngle),
          velocityVector.getY() + magChange * Math.sin(changeAngle));
    System.out.print("Drive Vector: ");
    System.out.println(driveVector);
    System.out.print("Velocity Vector: ");
    System.out.println(velocityVector);
    /* Drive */
    s_Swerve.drive(driveVector, rotationVal * SwerveConstants.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(), false);
  }
}
