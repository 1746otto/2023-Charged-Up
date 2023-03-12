package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexersubsystem;
import frc.robot.subsystems.Flapsubsystem;

public class LowGoalCommand extends CommandBase {
  private final Indexersubsystem m_indexerSubsystem;
  private final Flapsubsystem m_flapSubsystem;

  public LowGoalCommand(Indexersubsystem subsystem, Flapsubsystem subsystem1) {
    m_indexerSubsystem = subsystem;
    m_flapSubsystem = subsystem1;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.RunLowGoal();
    m_flapSubsystem.openFlap();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.turnOffShooter();
    m_flapSubsystem.closeFlap();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


