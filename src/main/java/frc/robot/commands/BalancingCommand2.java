package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancingCommand2 extends CommandBase {
  private final double kP = 0.22;
  private final double kD = 0.025;
  private final double kI = 0.0;

  private Swerve s_Swerve;
  private double xError;
  private double deltaError;
  private double prevError;
  private double totalError;
  private double speed;

  public BalancingCommand2(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

    addRequirements(s_Swerve);

  }

  @Override
  public void execute() {
    double[] velocities = new double[3];
    s_Swerve.gyro.getRawGyro(velocities);
    if (s_Swerve.gyro.getRoll() != 0) {
      xError = -(s_Swerve.gyro.getRoll());
      totalError += xError;
      deltaError = xError - prevError;
      System.out.println("Roll: " + xError);

      speed = kP * Math.sin(xError * Math.PI / 90) + kI * Math.sin(totalError * Math.PI / 120)
          + kD * velocities[0];

      if (speed > SwerveConstants.autonDriveSpeed) {
        speed = SwerveConstants.autonDriveSpeed;
      } else if (speed < -SwerveConstants.autonDriveSpeed) {
        speed = -SwerveConstants.autonDriveSpeed;
      }
      prevError = xError;

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
    System.out.println(velocities[0]);
    System.out.println(Math.floor(s_Swerve.gyro.getRoll() / 2.5) == 0.0);
    System.out.println(Math.floor(velocities[0]) == 0.0);
    return (Math.floor(s_Swerve.gyro.getRoll() / 2.5) == 0.0)
        && (Math.floor(velocities[0] / 2.0) == 0.0);
  }
}
