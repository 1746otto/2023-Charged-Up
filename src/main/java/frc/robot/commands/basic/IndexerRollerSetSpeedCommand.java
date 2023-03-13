package frc.robot.commands.basic;

import frc.robot.subsystems.IndexerRollerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerRollerSetSpeedCommand extends CommandBase {
  private final IndexerRollerSubsystem m_indexerRollerSubsystem;
  private final double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerRollerSetSpeedCommand(IndexerRollerSubsystem subsystem, double speed) {
    m_indexerRollerSubsystem = subsystem;
    addRequirements(subsystem);

    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerRollerSubsystem.setMotorSpeed(m_speed);
  }
}
