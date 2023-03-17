package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRunToRequestCommand extends CommandBase {
  private final ElevatorSubsystem m_elevator;
  private final double requestedPosition;
  private boolean needStop;
  private boolean needStopAtBoth;

  public ElevatorRunToRequestCommand(ElevatorSubsystem subsystem, double requestedPosition) {
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
    if (needStopAtBoth) {
      m_elevator.stopElevator();
    }
    if (needStop) {
      m_elevator.setPositionTo0();
    }
  }

  // Returns true when the command should end.
  // @Override
  public boolean isFinished() {
    needStop = m_elevator.beamBreakBroken();
    needStopAtBoth = (m_elevator.beamBreakBroken() || m_elevator.limitSwitchActivated());
    return needStopAtBoth;
  }
}
