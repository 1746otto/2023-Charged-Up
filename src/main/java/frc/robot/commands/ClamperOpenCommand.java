package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

public class ClamperOpenCommand extends CommandBase {
  private final PlacerSubsystem m_PlacerSubsystem;

  public ClamperOpenCommand(PlacerSubsystem subsystem) {
    m_PlacerSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_PlacerSubsystem.openClamper();
  }
}
