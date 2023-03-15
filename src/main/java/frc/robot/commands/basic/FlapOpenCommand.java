package frc.robot.commands.basic;

import frc.robot.subsystems.FlapSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FlapOpenCommand extends CommandBase {
  private final FlapSubsystem m_flapSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlapOpenCommand(FlapSubsystem subsystem) {
    m_flapSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flapSubsystem.openFlap();
  }
}
