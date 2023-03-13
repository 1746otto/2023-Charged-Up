package frc.robot.commands.basic;

import frc.robot.subsystems.IndexerRollerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerRollerStopCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerRollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerRollerStopCommand(IndexerRollerSubsystem subsystem) {
    m_indexerRollerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerRollerSubsystem.setMotorStoppedSpeed();
  }
}
