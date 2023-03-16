package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;

public class ClamperCloseCommand extends CommandBase {
  private final ClamperSubsystem m_ClamperSubsystem;

  public ClamperCloseCommand(ClamperSubsystem subsystem) {
    m_ClamperSubsystem = subsystem;
    addRequirements(m_ClamperSubsystem);
  }

  @Override
  public void initialize() {
    m_ClamperSubsystem.close();
    System.out.println("Clamper Closed: " + m_ClamperSubsystem.isClamperClosed());
  }

  @Override
  public boolean isFinished() {
    return m_ClamperSubsystem.isClamperClosed();
  }
}
