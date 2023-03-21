package frc.robot.commands;

import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.IntakeExtensionSubsystem;

public class IndexerCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerSubsystem;
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;
  private final ClamperSubsystem m_ClamperSubsystem;



  public IndexerCommand(IndexerRollerSubsystem m_IndexerRollerSubsystem,
      IndexerTreadSubsystem m_IndexerTreadSubsystem, ClamperSubsystem m_ClamperSubsystem) {
    this.m_indexerSubsystem = m_IndexerRollerSubsystem;
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    this.m_ClamperSubsystem = m_ClamperSubsystem;


    addRequirements(m_IndexerRollerSubsystem, m_IndexerTreadSubsystem);
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.setMotorIntakeSpeed();
    m_IndexerTreadSubsystem.setMotorIntakeSpeed();
    m_ClamperSubsystem.open();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setMotorStoppedSpeed();
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
    m_ClamperSubsystem.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return m_IndexerTreadSubsystem.beambreakBroken();
  }
}


