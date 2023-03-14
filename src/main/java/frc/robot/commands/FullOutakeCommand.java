package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ClamperSubsystem;

public class FullOutakeCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerSubsystem;
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;
  private final IntakeRollerSubsystem m_IntakeRollerSubsystem;
  private final ClamperSubsystem m_ClamperSubsystem;


  public FullOutakeCommand(IndexerRollerSubsystem m_IndexerRollerSubsystem,
      IntakeRollerSubsystem m_IntakeRollerSubsystem,
      IndexerTreadSubsystem m_IndexerTreadSubsystem,ClamperSubsystem m_ClamperSubsystem) {
    this.m_indexerSubsystem = m_IndexerRollerSubsystem;
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    this.m_IntakeRollerSubsystem = m_IntakeRollerSubsystem;
    this.m_ClamperSubsystem = m_ClamperSubsystem;

    addRequirements();
  }

  @Override
  public void initialize() {
    m_ClamperSubsystem.open();
    m_indexerSubsystem.setMotorOuttakeSpeed();
    m_IndexerTreadSubsystem.setMotorOuttakeSpeed();
    m_IntakeRollerSubsystem.setMotorOuttakeSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setMotorStoppedSpeed();
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
    m_IntakeRollerSubsystem.setMotorStoppedSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


