package frc.robot.commands.basic;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;

public class BalanceSpeedCommand extends CommandBase {

  @Override
  public void initialize() {
    SwerveConstants.maxSpeed *= .5;

  }

  @Override
  public void end(boolean interrupted) {
    SwerveConstants.maxSpeed *= 2.0;
  }
}
