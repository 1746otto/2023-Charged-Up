package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultRunToMin extends CommandBase {
  private CatapultSubsystem m_catapult;

  public CatapultRunToMin(CatapultSubsystem subsystem) {
    m_catapult = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_catapult.setServoToMin();
  }
}
