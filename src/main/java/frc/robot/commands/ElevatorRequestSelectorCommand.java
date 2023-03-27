package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRequestSelectorCommand extends CommandBase {
  private final ElevatorSubsystem m_elevator;
  private final double requestedPosition;
  private boolean needStop;
  private boolean needStopAtBoth;

  public ElevatorRequestSelectorCommand(ElevatorSubsystem subsystem, double requestedPosition) {
    m_elevator = subsystem;
    addRequirements(subsystem);
    this.requestedPosition = requestedPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorReq(requestedPosition);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
