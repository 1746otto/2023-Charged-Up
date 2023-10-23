package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.pipelineStates;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BackupAlignCommand extends CommandBase {
  VisionSubsystem m_visionSubsystem;
  Swerve m_swerve;
  private int targetAngle = 0;
  private Pigeon2 m_gyro;
  private pipelineStates state;
  private boolean shouldRotate = true;
  private boolean shouldAlign = false;
  Alliance m_allianceColor = DriverStation.getAlliance();
  private int allianceDirection = -1;

  private double getRotationSignal(double distFromTarget) {
    return Math.copySign(Math.abs(distFromTarget) / 90.0 * SwerveConstants.maxAngularVelocity / 3.0
        + 0.05 * SwerveConstants.maxAngularVelocity, distFromTarget);
  }

  private double getStrafeVal(double distFromTarget) {
    return Math.copySign(Math.abs(Math.sin(distFromTarget * 2)) * SwerveConstants.maxSpeed / 3.0
        + 0.05 * SwerveConstants.maxSpeed, distFromTarget * allianceDirection);
  }

  public BackupAlignCommand(VisionSubsystem vision, Swerve swerve, pipelineStates state) {
    m_visionSubsystem = vision;
    m_swerve = swerve;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionSubsystem, m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_allianceColor == Alliance.Red) {
      allianceDirection = 1;
    }
    targetAngle = (int) m_gyro.getYaw() / 180;
    targetAngle *= 180;
    m_visionSubsystem.setLeftState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shouldRotate) {
      m_swerve.drive(new Translation2d(), getRotationSignal(m_gyro.getYaw() - targetAngle), true,
          true);
      if (Math.abs(m_gyro.getYaw() - targetAngle) < 1.5) {
        shouldAlign = true;
        shouldRotate = false;
      }
    } else {
      m_swerve.drive(new Translation2d(0, getStrafeVal(m_visionSubsystem.getXOffsetLeft())),
          getRotationSignal(m_gyro.getYaw() - targetAngle), true, true);
      if (Math.abs(m_visionSubsystem.getXOffsetLeft()) < .5)
        shouldAlign = false;
    }


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(shouldAlign || shouldRotate);
  }
}
