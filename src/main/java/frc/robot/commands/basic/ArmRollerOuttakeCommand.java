package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRollersSubsystem;

public class ArmRollerOuttakeCommand extends CommandBase {
  private ArmRollersSubsystem m_Arm;

  public ArmRollerOuttakeCommand(ArmRollersSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.armRollerOuttake();
  }

  @Override
  public void end(boolean interrupted) {
    m_Arm.armRollerStop();
  }
}
