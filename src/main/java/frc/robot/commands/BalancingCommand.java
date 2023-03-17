package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancingCommand extends CommandBase {
  private final double kP = 0.5;
  // private final double kD = 0.1;

  private Swerve s_Swerve;
  private int direction;
  private double xError;
  private double speed;

  public BalancingCommand(Swerve s_Swerve, boolean isInCommunity) {
    this.s_Swerve = s_Swerve;

    direction = (isInCommunity) ? 1 : -1;

    addRequirements(s_Swerve);

  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      direction *= -1;
    }
    if (s_Swerve.getYaw().getDegrees() > 90)
      direction *= -1;
  }

  @Override
  public void execute() {
    if (s_Swerve.gyro.getRoll() != 0) {
      xError = -(s_Swerve.gyro.getRoll());
      System.out.println("Roll: " + xError);

      speed = kP * Math.sin(xError * Math.PI / 180.0);

      if (speed > SwerveConstants.autonDriveSpeed) {
        speed = SwerveConstants.autonDriveSpeed;
      } else if (speed < -SwerveConstants.autonDriveSpeed) {
        speed = -SwerveConstants.autonDriveSpeed;
      }
    }

    s_Swerve.drive(new Translation2d(speed, 0).times(SwerveConstants.maxSpeed).times(1), 0.0, true,
        false);
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.XLock();
  }

  @Override
  public boolean isFinished() {
    double[] velocities = new double[3];
    s_Swerve.gyro.getRawGyro(velocities);
    System.out.println(velocities);
    return (Math.floor(s_Swerve.gyro.getRoll()) == 0.0) && (Math.round(velocities[0]) == 0.0);
  }
}
