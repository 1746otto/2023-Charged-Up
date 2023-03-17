package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlungerSubsystem;

public class PlungerExtendCommand extends CommandBase {
  private final PlungerSubsystem m_plungerSubsystem;

  public PlungerExtendCommand(PlungerSubsystem subsystem) {
    m_plungerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_plungerSubsystem.extend();
  }

  @Override
  public void end(boolean interrupted) {
    m_plungerSubsystem.Afterdelay();
  }

  @Override
  public boolean isFinished() {
    return m_plungerSubsystem.isExtended();
  }
}
