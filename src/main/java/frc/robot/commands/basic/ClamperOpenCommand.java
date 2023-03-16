package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;

public class ClamperOpenCommand extends CommandBase {
  private final ClamperSubsystem m_ClamperSubsystem;

  public ClamperOpenCommand(ClamperSubsystem subsystem) {
    m_ClamperSubsystem = subsystem;
    addRequirements(m_ClamperSubsystem);
  }

  @Override
  public void initialize() {
    m_ClamperSubsystem.open();
    System.out.println("Clamper Closed: " + m_ClamperSubsystem.isClamperClosed());
  }

  @Override
  public boolean isFinished() {
    return !m_ClamperSubsystem.isClamperClosed();
  }
}
