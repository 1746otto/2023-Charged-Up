package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroOutElevatorCommand extends CommandBase {
  private final ElevatorSubsystem m_elevator;

  public ZeroOutElevatorCommand(ElevatorSubsystem subsystem) {
    m_elevator = subsystem;
  }

  @Override
  public void execute() {
    m_elevator.elevatorRunDown();
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopElevator();
    m_elevator.setPositionTo0();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.beamBreakBroken();
  }
}
