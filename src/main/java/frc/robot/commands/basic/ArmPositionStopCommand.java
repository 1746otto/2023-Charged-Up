package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPositionSubsystem;

public class ArmPositionStopCommand extends CommandBase {
  private ArmPositionSubsystem m_Arm;

  public ArmPositionStopCommand(ArmPositionSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_Arm.armStop();
  }
}
