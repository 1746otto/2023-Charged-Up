package frc.robot.commands.basic;

import frc.robot.subsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerRollerIntakeCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerRollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerRollerIntakeCommand(IndexerRollerSubsystem subsystem) {
    m_indexerRollerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerRollerSubsystem.setMotorIntakeSpeed();
  }

}
