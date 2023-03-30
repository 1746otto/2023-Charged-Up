package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TrajectoryDirectionTesting extends CommandBase {
  private double kSpeed = SwerveConstants.autonDriveSpeed;

  private Swerve m_Swerve;
  private int direction = 1;
  private int count;


  public TrajectoryDirectionTesting(Swerve m_Swerve) {
    this.m_Swerve = m_Swerve;
    addRequirements(m_Swerve);
    count = 0;

  }

  public void initialize() {
    count = 0;
  }

  @Override
  public void execute() {
    m_Swerve.drive(new Translation2d(kSpeed * direction, 0).times(SwerveConstants.maxSpeed / 6),
        0.0, false, false);
    count++;
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return (count >= 500);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    m_Swerve.drive(new Translation2d(0, 0), 0.0, false, false);
  }
}
