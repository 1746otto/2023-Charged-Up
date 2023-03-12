package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexersubsystem;

public class IndexerCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexersubsystem m_indexerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerCommand(Indexersubsystem subsystem) {
    m_indexerSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.runAllMotors();
    System.out.println("ran");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println(m_subsystem.beambreakBroken());
    m_indexerSubsystem.turnOffIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.beambreakBroken();
  }


}

