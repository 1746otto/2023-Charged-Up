package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LedConeCommand extends CommandBase {
  private LEDSubsystem m_LedSubsystem;

  public LedConeCommand(LEDSubsystem subsystem) {
    m_LedSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_LedSubsystem.setLedtoCone();
  }

  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.setLedOff();
  }
}
