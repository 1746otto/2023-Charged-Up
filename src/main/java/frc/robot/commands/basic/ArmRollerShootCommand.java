package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRollersSubsystem;

public class ArmRollerShootCommand extends CommandBase {
  private ArmRollersSubsystem m_Arm;

  public ArmRollerShootCommand(ArmRollersSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.armRollerShoot();
  }

  @Override
  public void end(boolean interrupted) {
    m_Arm.armRollerStop();
  }
}
