package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancingCommand extends CommandBase {
  private final double kP = 0.01;
  // private final double kD = 0.1;

  private Swerve s_Swerve;
  private BooleanSupplier robotCentricSup;
  private double xError;
  private double speed;

  public BalancingCommand(Swerve s_Swerve, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    if (s_Swerve.gyro.getRoll() != 0) {
      xError = -(s_Swerve.gyro.getRoll());
      System.out.println("Roll: " + xError);

      speed = kP * xError;

      if (speed > SwerveConstants.autonDriveSpeed) {
        speed = SwerveConstants.autonDriveSpeed;
      } else if (speed < -SwerveConstants.autonDriveSpeed) {
        speed = -SwerveConstants.autonDriveSpeed;
      }
    }

    s_Swerve.drive(new Translation2d(speed, 0).times(SwerveConstants.maxSpeed), 0.0,
        robotCentricSup.getAsBoolean(), true);
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 180.0, robotCentricSup.getAsBoolean(), true);
  }

  @Override
  public boolean isFinished() {
    return (s_Swerve.gyro.getRoll() == 0);
  }
}
