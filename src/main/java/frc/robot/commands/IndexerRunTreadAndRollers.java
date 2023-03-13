package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.basic.IndexerRollerIntakeCommand;
import frc.robot.commands.basic.IndexerTreadIntakeCommand;

public class IndexerRunTreadAndRollers extends ParallelCommandGroup {
  public IndexerRunTreadAndRollers(IndexerRollerIntakeCommand m_IndexerRollerIntakeCommand,
      IndexerTreadIntakeCommand m_IndexerTreadIntakeCommand) {
    addCommands(m_IndexerRollerIntakeCommand, m_IndexerTreadIntakeCommand);
  }
}
