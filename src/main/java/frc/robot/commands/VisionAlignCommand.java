// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class VisionAlignCommand extends CommandBase {
  private final VisionSubsystem m_visionSubsystem;
  private final ExampleSubsystem m_driveSubsystem;
  private int m_tagID;
  private double m_xOffset; //in degrees. 50 for nothin. Kinda jank, could probably use something better but it is fine.
  private double targetAngle;
  private int direction = 1;
  private Pigeon2 m_gyro;
  private double initialAngle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAlignCommand(VisionSubsystem vision, ExampleSubsystem drive) {
    m_driveSubsystem = drive;
    m_visionSubsystem = vision;
    m_gyro = m_driveSubsystem.pigeon;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tagID = m_visionSubsystem.getTagID();
    if (m_tagID != 0) {
        if (m_tagID <= 4) {
            targetAngle = 0;
        } 
        else {
            targetAngle = 180;
        }
    }
    if (m_gyro.getYaw() % 360 - targetAngle >= 0) {
        direction = -1;
    }
    initialAngle = m_gyro.getYaw() % 360;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationalVal = (m_gyro.getYaw() % 360 -targetAngle < 45)
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
