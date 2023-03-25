package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRollersSubsystem;

public class ArmRollerStopCommand extends CommandBase {
  private ArmRollersSubsystem m_Arm;

  public ArmRollerStopCommand(ArmRollersSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_Arm.armRollerStop();
  }
}
