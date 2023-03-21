package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import frc.robot.subsystems.IntakeExtensionSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ClamperSubsystem;

public class FullOutakeCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerRollerSubsystem;
  private final IndexerTreadSubsystem m_IndexerTreadSubsystem;
  private final IntakeRollerSubsystem m_IntakeRollerSubsystem;
  private final ClamperSubsystem m_ClamperSubsystem;
  private final IntakeExtensionSubsystem m_intakeExtensionSubsystem;


  public FullOutakeCommand(IndexerRollerSubsystem m_IndexerRollerSubsystem,
      IntakeRollerSubsystem m_IntakeRollerSubsystem, IndexerTreadSubsystem m_IndexerTreadSubsystem,
      ClamperSubsystem m_ClamperSubsystem, IntakeExtensionSubsystem intakeextension) {
    this.m_indexerRollerSubsystem = m_IndexerRollerSubsystem;
    this.m_IndexerTreadSubsystem = m_IndexerTreadSubsystem;
    this.m_IntakeRollerSubsystem = m_IntakeRollerSubsystem;
    this.m_ClamperSubsystem = m_ClamperSubsystem;
    this.m_intakeExtensionSubsystem = intakeextension;

    addRequirements(this.m_ClamperSubsystem, this.m_IndexerTreadSubsystem,
        this.m_IntakeRollerSubsystem, this.m_indexerRollerSubsystem,
        this.m_intakeExtensionSubsystem);
  }

  @Override
  public void initialize() {
    m_ClamperSubsystem.open();
    m_indexerRollerSubsystem.setMotorOuttakeSpeed();
    m_IndexerTreadSubsystem.setMotorOuttakeSpeed();
    m_IntakeRollerSubsystem.setMotorOuttakeSpeed();
    m_intakeExtensionSubsystem.setMotorExtensionSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerRollerSubsystem.setMotorStoppedSpeed();
    m_IndexerTreadSubsystem.setMotorStoppedSpeed();
    m_IntakeRollerSubsystem.setMotorStoppedSpeed();
    m_intakeExtensionSubsystem.setMotorRetractionSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


