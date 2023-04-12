package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPositionSubsystem;

public class ArmRequestSelectorCommand extends CommandBase {
  private ArmPositionSubsystem m_Arm;
  private double request;

  public ArmRequestSelectorCommand(ArmPositionSubsystem subsystem, double requestedPosition) {
    m_Arm = subsystem;
    request = requestedPosition;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_Arm.setRequest(request);
  }

  @Override
  public boolean isFinished() {
    return m_Arm.armReqisCorrect(request);
  }
}
