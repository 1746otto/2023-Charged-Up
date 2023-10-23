package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LedCubeCommand extends CommandBase {
  private LEDSubsystem m_LedSubsystem;

  public LedCubeCommand(LEDSubsystem subsystem) {
    m_LedSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_LedSubsystem.setLEDToCube();
  }

  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.setLedOff();
  }
}
