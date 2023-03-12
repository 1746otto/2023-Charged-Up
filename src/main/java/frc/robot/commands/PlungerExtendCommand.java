package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

public class PlungerExtendCommand extends CommandBase {
  private final PlacerSubsystem m_placerSubsystem;

  public PlungerExtendCommand(PlacerSubsystem subsystem) {
    m_placerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_placerSubsystem.extendPlunger();
  }
}
