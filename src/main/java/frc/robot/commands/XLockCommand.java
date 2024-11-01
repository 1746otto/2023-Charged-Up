package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class XLockCommand extends CommandBase {
  private Swerve s_Swerve;

  public XLockCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    s_Swerve.drive(new Translation2d(), 0, true, true);
    s_Swerve.XLock();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
