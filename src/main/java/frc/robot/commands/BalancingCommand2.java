package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.Vector;

public class BalancingCommand2 extends CommandBase {
  private Swerve s_Swerve;
  private Rotation3d RollPitchVector;


  public BalancingCommand2(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    System.out.println(new Rotation3d(0, 0, 0).getQuaternion().toRotationVector());
    System.out.println(new Rotation3d(0, 0, Math.PI / 4).getQuaternion().toRotationVector());
    System.out.println(new Rotation3d(0, Math.PI / 4, 0).getQuaternion().toRotationVector());
    System.out.println(new Rotation3d(Math.PI / 4, 0, 0).getQuaternion().toRotationVector());
  }

  @Override
  public void execute() {
    RollPitchVector =
        new Rotation3d(s_Swerve.gyro.getRoll(), s_Swerve.gyro.getPitch(), s_Swerve.gyro.getYaw());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
