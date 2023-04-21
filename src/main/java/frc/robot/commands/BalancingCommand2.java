package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalancingCommand2 extends CommandBase {
  private final double kP = (1 / 300.0);
  private final double kD = (1 / 4000.0);
  private final double kI = 0.0;

  private Swerve s_Swerve;
  private double xError;
  private double deltaError;
  private double prevError;
  private double totalError;
  private double speed;
  private double driveDirection;

  public BalancingCommand2(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

    addRequirements(s_Swerve);


    if (DriverStation.getAlliance() == Alliance.Blue) {
      driveDirection = -1;

    } else {
      driveDirection = 1;
    }

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

      speed = kP * xError + kI * totalError + (-kD) * velocities[0];

      if (speed > SwerveConstants.autonDriveSpeed) {
        speed = SwerveConstants.autonDriveSpeed;
      } else if (speed < -SwerveConstants.autonDriveSpeed) {
        speed = -SwerveConstants.autonDriveSpeed;
      }
      prevError = xError;

    }

    s_Swerve.drive(new Translation2d(speed * driveDirection, 0).times(13.5).times(1), 0.0, true,
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
    return (Math.floor(s_Swerve.gyro.getRoll() / 7.5) == 0.0);
  }
}
