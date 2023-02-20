// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

import java.io.IOException;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */

public class VisionAlignCommand extends CommandBase {
  private final VisionSubsystem m_visionSubsystem;
  private final Swerve m_swerve;
  private int targetType = 0; //0 is apriltags for 1 is for cones.
  private Pigeon2 m_gyro;
  private boolean isCube;
  Alliance m_allianceColor;
  private double initialAngle;
  private final double[] AprilTagYValues = new double[]{1.071626, 2.748026, 4.424426, 6.749796, 6.749796, 4.424426, 2.748026, 1.071626};
  private final double[] AprilTagXValues = new double[]{15.513558, 15.513558, 15.513558, 16.178784, 0.36195, 1.02743, 1.02743, 1.02743};
  private final double nodeDepth = Units.inchesToMeters(16 + 12); //includes width of the robot with bumpers.
  private final double[] poleYValues = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  private double targetY;
  private double direction = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAlignCommand(VisionSubsystem vision, Swerve Swerve) {
    m_swerve = Swerve;
    m_visionSubsystem = vision;
    m_gyro = Swerve.gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionSubsystem, m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = (m_gyro.getYaw() + 180) % 360 - 180;
    isCube = true;
    double closest = 10;
    m_allianceColor = DriverStation.getAlliance();
    if (m_allianceColor == Alliance.Red)
      direction = -1;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //not done
      m_swerve.drive(new Translation2d(), direction, isCube, isCube);
    }


  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
