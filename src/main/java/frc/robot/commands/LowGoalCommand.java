package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;

public class LowGoalCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerSubsystem;
  private final FlapSubsystem m_FlapSubsystem;
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;

  public LowGoalCommand(IndexerRollerSubsystem m_IndexerRollerSubsystem,
      FlapSubsystem m_FlapSubsystem, IndexerTreadSubsystem m_IndexerTreadSubsystem) {
    this.m_indexerSubsystem = m_IndexerRollerSubsystem;
    this.m_FlapSubsystem = m_FlapSubsystem;
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    addRequirements();
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.setMotorIntakeSpeed();
    m_FlapSubsystem.openFlap();
    m_IndexerTreadSubsystem.setMotorScoreSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setMotorStoppedSpeed();
    m_FlapSubsystem.closeFlap();
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


