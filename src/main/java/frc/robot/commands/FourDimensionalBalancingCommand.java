package frc.robot.commands;

import frc.lib.math.Conversions;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

public class FourDimensionalBalancingCommand extends CommandBase {
  private Swerve s_Swerve;
  private Rotation3d RollPitchVector;
  private Vector<N3> p1;
  private Vector<N3> p2;
  private Vector<N3> robotNormVector;
  private double[] temp;
  private Quaternion robotQuaternion;


  public FourDimensionalBalancingCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    temp = new double[4];
    robotQuaternion = new Quaternion();
  }

  @Override
  public void initialize() {
    System.out.println(new Rotation3d(0, 0, 0).getQuaternion());
    System.out.println(new Rotation3d(0, 0, Math.PI / 4).getQuaternion());
    System.out.println(new Rotation3d(Math.PI, Math.PI / 4, 0).getQuaternion().inverse());
    System.out.println(new Rotation3d(Math.PI / 4, 0, 0).getQuaternion());
    p1 = VecBuilder.fill(0, 0, 0);
    p2 = VecBuilder.fill(0, 0, 0);
  }

  @Override
  public void execute() {
    RollPitchVector = new Rotation3d(Math.PI * s_Swerve.gyro.getRoll() / 180.0,
        Math.PI * s_Swerve.gyro.getPitch() / 180.0, Math.PI * s_Swerve.gyro.getYaw() / 180.0);
    s_Swerve.gyro.get6dQuaternion(temp);
    p1.set(0, 0, temp[0] * temp[0] + temp[1] * temp[1] - temp[3] * temp[3] - temp[2] * temp[2]);
    p1.set(1, 0, 2 * temp[1] * temp[2] + 2 * temp[0] * temp[3]);
    p1.set(2, 0, 2 * temp[1] * temp[3] - 2 * temp[0] * temp[2]);
    p2.set(0, 0, -2 * temp[3] * temp[0] + 2 * temp[2] * temp[1]);
    p2.set(1, 0, temp[2] * temp[2] - temp[3] * temp[3] + temp[0] * temp[0] - temp[1] * temp[1]);
    p2.set(2, 0, 2 * temp[2] * temp[3] + 2 * temp[0] * temp[1]);
    robotNormVector.set(0, 0, p1.get(1, 0) * p2.get(2, 0) - p1.get(2, 0) * p2.get(1, 0));
    robotNormVector.set(1, 0, p1.get(2, 0) * p2.get(0, 0) - p1.get(0, 0) * p2.get(2, 0));
    robotNormVector.set(2, 0, p1.get(0, 0) * p2.get(2, 0) - p1.get(1, 0) * p2.get(0, 0));
    if (robotNormVector.get(2, 0) < 0)
      robotNormVector = robotNormVector.times(-1);
    double angle = Math.acos(robotNormVector.get(2, 0));
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
