package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

public class ClamperCloseCommand extends CommandBase {
  private final PlacerSubsystem m_PlacerSubsystem;

  public ClamperCloseCommand(PlacerSubsystem subsystem) {
    m_PlacerSubsystem = subsystem;
    addRequirements(m_PlacerSubsystem);
  }

  @Override
  public void initialize() {
    m_PlacerSubsystem.closeClamper();
  }
}
