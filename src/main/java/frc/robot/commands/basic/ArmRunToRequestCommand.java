package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPositionSubsystem;

public class ArmRunToRequestCommand extends CommandBase {
  private ArmPositionSubsystem m_Arm;
  private double request;

  public ArmRunToRequestCommand(ArmPositionSubsystem subsystem, double requestedPosition) {
    m_Arm = subsystem;
    request = requestedPosition;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.armToCustom(request);
  }

  @Override
  public void end(boolean interrupted) {}

  // @Override
  // public boolean isFinished() {
  // return m_Arm.isAtOrigin();
  // }
}
