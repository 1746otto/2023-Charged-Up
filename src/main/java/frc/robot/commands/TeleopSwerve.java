package frc.robot.commands;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

  }

  public boolean joystickBeingUsed(double rotationalVal) {
    if (rotationalVal == 0) {
      return false;
    }
    return true;
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

    /* Drive */
    s_Swerve.drive(new Translation2d(Math.pow(driveVector.getNorm(), 1), driveVector.getAngle()),
        rotationVal * SwerveConstants.maxAngularVelocity, !robotCentricSup.getAsBoolean(), false);
  }
}
