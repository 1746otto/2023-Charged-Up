package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlungerSubsystem;

public class PlungerRetractCommand extends CommandBase {
  private final PlungerSubsystem m_plungerSubsystem;

  public PlungerRetractCommand(PlungerSubsystem subsystem) {
    m_plungerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_plungerSubsystem.retract();
  }

  @Override
  public void end(boolean interrupted) {
    m_plungerSubsystem.Afterdelay();
  }

  @Override
  public boolean isFinished() {
    return !m_plungerSubsystem.isExtended();
  }
}
