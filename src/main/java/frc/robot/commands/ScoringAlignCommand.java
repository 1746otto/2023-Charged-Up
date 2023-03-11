// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

import java.io.IOException;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */

public class ScoringAlignCommand extends CommandBase {
  private final Swerve m_swerve;
  private Pigeon2 m_gyro;
  private boolean isCube;
  private double targetAngle = 0.0;

  //this will need to be corrected by an offset for the specific robot so we can go the correct distance.
  private double targetX = 1.02743;
  private final double[] AprilTagYValues = new double[]{1.071626, 2.748026, 4.424426, 6.749796, 6.749796, 4.424426, 2.748026, 1.071626};
  private final double[] AprilTagXValues = new double[]{15.513558, 15.513558, 15.513558, 16.178784, 0.36195, 1.02743, 1.02743, 1.02743};
  private final double nodeDepth = Units.inchesToMeters(16 + 12); //includes width of the robot with bumpers.
  private final double[] poleYValues = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  private double alliance = 1;
  private final double halfNodeSpacing = .57/2.0;
  private double closest;
  private double stage;
  private boolean isAuton;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoringAlignCommand(Swerve Swerve, boolean isAuton) {
    m_swerve = Swerve;
    m_gyro = Swerve.gyro;
    this.isAuton = isAuton;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // does everything we want while clamping to 360 deg (m_gyro.getYaw() + 180) % 360 - 180;
    //this is commented because we are testing without an intitial pose, and we don't have a field.
    isCube = true;
    closest = 10;
    if (DriverStation.getAlliance() == Alliance.Red) {
      alliance = -1;
      targetX = 15.513558;
      targetAngle = 180.0;
    }
    if (isCube) {
      for(int i = 0; i < 4; i++) {
        if (Math.abs(m_swerve.getPose().getY() - AprilTagYValues[i]) < closest) {
          closest = AprilTagYValues[i];
        }
      }
    }
    else {
      for(int i = 0; i < 7; i++) {
        if (Math.abs(m_swerve.getPose().getY() - poleYValues[i]) < closest) {
          closest = poleYValues[i];
        }
      }
    }
    //this if for testing purposes
    // closest = 0.15;
    // targetX = 0.1;
    // targetAngle = 25;
    stage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage == 0) {
      m_swerve.drive(
        new Translation2d(0, Math.copySign(Math.sqrt(Math.abs((closest - m_swerve.getPose().getY())/halfNodeSpacing)), closest - m_swerve.getPose().getY())*alliance)
          .times(SwerveConstants.maxSpeed*0.025), 
        Math.copySign(Math.sqrt(Math.abs(targetAngle - (m_gyro.getYaw() + 180) % 360 - 180)/45.0), targetAngle - (m_gyro.getYaw() + 180) % 360 - 180),
        true, true);
      if (Math.abs(closest - m_swerve.getPose().getY()) < 0.01)
        stage = 1;
    } else if (stage == 1) {
      m_swerve.drive(new Translation2d(Math.copySign(Math.sqrt(Math.abs((targetX - m_swerve.getPose().getX()))/0.2032), targetX - m_swerve.getPose().getX()), 0), 0, true, true);
      if (Math.abs(targetX - m_swerve.getPose().getX()) < 0.02)
        stage = 2;
    }
    else if (stage == 2) {
      m_swerve.drive(
        new Translation2d(
          Math.copySign(Math.sqrt(Math.abs((targetX - m_swerve.getPose().getX()))/0.2032), targetX - m_swerve.getPose().getX()), 
          Math.copySign(Math.sqrt(Math.abs((closest - m_swerve.getPose().getY())/halfNodeSpacing)), closest - m_swerve.getPose().getY())*alliance)
            .times(SwerveConstants.maxSpeed*0.025), 
        Math.copySign(Math.sqrt(Math.abs(targetAngle - (m_gyro.getYaw() + 180) % 360 - 180)/45.0), targetAngle - (m_gyro.getYaw() + 180) % 360 - 180),
        true, true);
    }
    
  }


  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuton)
      return (Math.abs(closest - m_swerve.getPose().getY()) < 0.01) && (Math.abs(targetX - m_swerve.getPose().getX()) < 0.02) && (Math.abs(targetAngle - (m_gyro.getYaw() + 180) % 360 - 180) < 1.5);
    return false;
  }
}
