package frc.robot.commands.basic;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeDunkerSubsytem;

public class TheDunkCommand extends CommandBase {
  private ConeDunkerSubsytem m_DunkerSubsytem;
  private boolean current;

  private enum DunkerState {
    Down, Stop, Up
  }

  private DunkerState State;

  public TheDunkCommand(ConeDunkerSubsytem subsystem) {
    m_DunkerSubsytem = subsystem;
    addRequirements(subsystem);
    State = DunkerState.Down;
  }

  @Override
  public void execute() {
    if (State == DunkerState.Down) {
      m_DunkerSubsytem.theDunk();
      if (m_DunkerSubsytem.DownPosition()) {
        State = DunkerState.Up;
      }
    } else if (State == DunkerState.Stop) {
      m_DunkerSubsytem.dunkStop();
    } else if (State == DunkerState.Up) {
      m_DunkerSubsytem.theReverseDunk();
      if (m_DunkerSubsytem.UpPosition()) {
        State = DunkerState.Stop;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_DunkerSubsytem.dunkStop();
  }


  @Override
  public boolean isFinished() {
    return State == DunkerState.Stop;
  }

}
