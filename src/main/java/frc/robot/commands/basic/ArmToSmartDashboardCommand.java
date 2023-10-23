package frc.robot.commands.basic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPositionSubsystem;

public class ArmToSmartDashboardCommand extends CommandBase {
  private ArmPositionSubsystem m_Arm;
  private double request;

  public ArmToSmartDashboardCommand(ArmPositionSubsystem subsystem) {
    m_Arm = subsystem;
    request = m_Arm.getRequestedPosition();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_Arm.setRequest(request);
  }

  @Override
  public void execute() {
    if (SmartDashboard.getNumber("Target Position", request) != request)
      request = Math.max(Math.min(SmartDashboard.getNumber("Target Position", request), 0), -18000);
    m_Arm.setRequest(request);

  }

  @Override
  public boolean isFinished() {
    return m_Arm.armReqisCorrect(request);
  }
}
