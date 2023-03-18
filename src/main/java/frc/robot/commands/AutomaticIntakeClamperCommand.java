package frc.robot.commands;

import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.IntakeExtensionSubsystem;

public class AutomaticIntakeClamperCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerSubsystem;
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;
  private final IntakeRollerSubsystem m_IntakeRollerSubsystem;
  private final ClamperSubsystem m_ClamperSubsystem;
  private final IntakeExtensionSubsystem m_IntakeExtensionSubsystem;


  public AutomaticIntakeClamperCommand(IndexerRollerSubsystem m_IndexerRollerSubsystem,
      IndexerTreadSubsystem m_IndexerTreadSubsystem, IntakeRollerSubsystem m_IntakeRollerSubsystem,
      ClamperSubsystem m_ClamperSubsystem, IntakeExtensionSubsystem m_intakeExtensionSubsystem) {
    this.m_indexerSubsystem = m_IndexerRollerSubsystem;
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    this.m_IntakeRollerSubsystem = m_IntakeRollerSubsystem;
    this.m_ClamperSubsystem = m_ClamperSubsystem;
    this.m_IntakeExtensionSubsystem = m_intakeExtensionSubsystem;

    addRequirements();
  }

  @Override
  public void initialize() {
    m_IntakeRollerSubsystem.setMotorIntakeSpeed();
    m_indexerSubsystem.setMotorIntakeSpeed();
    m_IndexerTreadSubsystem.setMotorScoreSpeed();
    m_ClamperSubsystem.open();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeRollerSubsystem.setMotorStoppedSpeed();
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
    m_indexerSubsystem.setMotorStoppedSpeed();
    m_ClamperSubsystem.close();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return m_IndexerTreadSubsystem.beambreakBroken();
  }
}


