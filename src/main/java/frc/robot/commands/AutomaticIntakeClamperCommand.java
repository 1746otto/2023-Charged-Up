package frc.robot.commands;

import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClamperSubsystem;

public class AutomaticIntakeClamperCommand extends CommandBase {
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;
  private final IntakeRollerSubsystem m_IntakeRollerSubsystem;
  private final ClamperSubsystem m_ClamperSubsystem;
  private final IndexerRollerSubsystem m_IndexerRollerSubsystem;


  public AutomaticIntakeClamperCommand(IndexerRollerSubsystem IndexerRollerSubsystem,
      IndexerTreadSubsystem m_IndexerTreadSubsystem, ClamperSubsystem m_ClamperSubsystem,
      IntakeRollerSubsystem m_IntakeRollerSubsystem) {
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    this.m_ClamperSubsystem = m_ClamperSubsystem;
    this.m_IntakeRollerSubsystem = m_IntakeRollerSubsystem;
    m_IndexerRollerSubsystem = m_IndexerRollerSubsystem;
    addRequirements();
  }

  @Override
  public void initialize() {
    m_IntakeRollerSubsystem.setMotorIntakeSpeed();
    m_IndexerTreadSubsystem.setMotorScoreSpeed();
    m_ClamperSubsystem.open();
    m_IndexerRollerSubsystem.setMotorIntakeSpeed();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
    m_IntakeRollerSubsystem.setMotorStoppedSpeed();
    m_ClamperSubsystem.close();
    m_IndexerRollerSubsystem.setMotorStoppedSpeed();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return m_IndexerTreadSubsystem.beambreakBroken();
  }
}


