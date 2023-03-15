package frc.robot.commands.basic;

import frc.robot.subsystems.IndexerTreadSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerTreadSetSpeedCommand extends CommandBase {
  private final IndexerTreadSubsystem m_indexerTreadSubsystem;
  private final double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerTreadSetSpeedCommand(IndexerTreadSubsystem subsystem, double speed) {
    m_indexerTreadSubsystem = subsystem;
    addRequirements(subsystem);

    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerTreadSubsystem.setMotorSpeed(m_speed);
  }
}
