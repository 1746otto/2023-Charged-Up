package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class DriveBackTo5DegreesCommand extends CommandBase {
  private double kSpeed = SwerveConstants.autonDriveSpeed * 0.5;

  private Swerve s_Swerve;
  private double initRoll;
  private int direction = 1;

  public DriveBackTo5DegreesCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    initRoll = s_Swerve.gyro.getRoll();
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      direction = -1;
    }
  }

  @Override
  public void execute() {
    s_Swerve.drive(new Translation2d(-kSpeed * direction, 0).times(SwerveConstants.maxSpeed), 0.0,
        true, false);
    System.out.println("Roll: " + s_Swerve.gyro.getRoll());
  }

  @Override
  public boolean isFinished() {
    return Math.abs(s_Swerve.gyro.getRoll() - initRoll) > 13;
  }
}
