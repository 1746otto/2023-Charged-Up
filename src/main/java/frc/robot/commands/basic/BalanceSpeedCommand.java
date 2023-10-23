package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;

public class BalanceSpeedCommand extends CommandBase {

  @Override
  public void initialize() {
    SwerveConstants.maxSpeed *= .125;
    SwerveConstants.maxAngularVelocity *= 0.125;

  }

  @Override
  public void end(boolean interrupted) {
    SwerveConstants.maxSpeed *= 8.0;
    SwerveConstants.maxAngularVelocity *= 8.0;
  }
}
