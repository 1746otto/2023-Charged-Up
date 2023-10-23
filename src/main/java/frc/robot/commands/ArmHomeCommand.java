package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPositionSubsystem;

public class ArmHomeCommand extends CommandBase {
  ArmPositionSubsystem armPositionSubsystem;

  public ArmHomeCommand(ArmPositionSubsystem armPositionSubsystem) {
    this.armPositionSubsystem = armPositionSubsystem;
  }

  @Override
  public void initialize() {
    armPositionSubsystem.disablePID();
  }

  @Override
  public void execute() {
    armPositionSubsystem.setHomeSpeed();
  }

  @Override
  public void end(boolean interrupted) {
    armPositionSubsystem.zeroEncoder();
    armPositionSubsystem.enablePID();
    armPositionSubsystem.armToRequest(0);
  }
}
