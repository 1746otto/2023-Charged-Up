package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultRunToMax extends CommandBase {
  private CatapultSubsystem m_catapult;

  public CatapultRunToMax(CatapultSubsystem subsystem) {
    m_catapult = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_catapult.setServoToMax();
  }
}
