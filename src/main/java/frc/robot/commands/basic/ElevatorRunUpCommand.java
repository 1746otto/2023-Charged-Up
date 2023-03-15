package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRunUpCommand extends CommandBase {
  private final ElevatorSubsystem m_elevator;

  public ElevatorRunUpCommand(ElevatorSubsystem subsystem) {
    m_elevator = subsystem;
  }

  @Override
  public void execute() {
    m_elevator.elevatorRunUp();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.limitSwitchActivated();
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopElevator();;
  }
}
