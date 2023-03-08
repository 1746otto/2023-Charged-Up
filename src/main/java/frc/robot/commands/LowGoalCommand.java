package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexersubsystem;
import frc.robot.subsystems.Flapsubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowGoalCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexersubsystem m_subsystem;
  private final Flapsubsystem m_subsystem1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LowGoalCommand(Indexersubsystem subsystem, Flapsubsystem subsystem1) {
    m_subsystem = subsystem;
    m_subsystem1 = subsystem1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  @Override
  public void initialize(){
    m_subsystem.RunLowGoal();
    m_subsystem1.openFlap();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.turnOffShooter();
    m_subsystem1.closeFlap();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


