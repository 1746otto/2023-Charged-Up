package frc.robot.commands.basic;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;

public class NormalSpeedCommand extends CommandBase {

  @Override
  public void initialize() {
    SwerveConstants.maxSpeed *= 2;
    SwerveConstants.driveNeutralMode = NeutralMode.Coast;

  }
}
