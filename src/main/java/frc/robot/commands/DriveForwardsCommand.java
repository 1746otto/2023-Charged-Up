package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveForwardsCommand extends CommandBase {
  private double kSpeed = SwerveConstants.autonDriveSpeed;
  private Swerve s_Swerve;
  private Timer time;
  private int direction = 1;

  public DriveForwardsCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    time = new Timer();
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Red)
      direction = -1;
    time.start();
  }

  @Override
  public void execute() {
    s_Swerve.drive(new Translation2d(kSpeed * direction, 0).times(SwerveConstants.maxSpeed), 0.0,
        true, false);
  }

  @Override
  public boolean isFinished() {
    return (time.hasElapsed(2.0));
  }
}
