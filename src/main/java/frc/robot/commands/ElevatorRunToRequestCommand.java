package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRunToRequestCommand extends CommandBase {
  private final ElevatorSubsystem m_elevator;
  private final int requestedPosition;

  public ElevatorRunToRequestCommand(ElevatorSubsystem subsystem, int requestedPosition) {
    m_elevator = subsystem;
    addRequirements(subsystem);
    this.requestedPosition = requestedPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    m_elevator.runToRequest(requestedPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopElevator();
    m_elevator.setPositionTo0();
  }

  // Returns true when the command should end.
  // @Override
  public boolean isFinished() {
    return (m_elevator.beamBreakBroken() || m_elevator.limitSwitchActivated());
  }
}
