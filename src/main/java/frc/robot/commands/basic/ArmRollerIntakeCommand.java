package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRollersSubsystem;

public class ArmRollerIntakeCommand extends CommandBase {
  private ArmRollersSubsystem m_Arm;
  private boolean current;

  public ArmRollerIntakeCommand(ArmRollersSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.armRollerIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_Arm.armRollerStow();

  }

  @Override
  public boolean isFinished() {
    current = m_Arm.currentBroken();
    return current;
  }
}
