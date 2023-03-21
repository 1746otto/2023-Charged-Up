package frc.robot.commands.basic;

import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerTreadStopCommand extends CommandBase {
  private final IndexerTreadSubsystem m_indexerTreadSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerTreadStopCommand(IndexerTreadSubsystem subsystem) {
    m_indexerTreadSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerTreadSubsystem.setMotorStoppedSpeed();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
