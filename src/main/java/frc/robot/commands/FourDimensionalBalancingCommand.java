package frc.robot.commands;

import frc.lib.math.Conversions;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

public class FourDimensionalBalancingCommand extends CommandBase {
  private Swerve s_Swerve;
  private double[] p1;
  private double[] p2;
  private double zcross;
  private double[] temp;
  private int direction;


  public FourDimensionalBalancingCommand(Swerve s_Swerve, boolean isInCommunity) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    temp = new double[4];
    p1 = new double[2];
    p2 = new double[2];
    direction = (isInCommunity) ? 1 : -1;
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      direction *= -1;
    }
  }

  void RPYToQdeg(double roll, double pitch, double yaw, double[] q) {
    roll *= Math.PI / 180.0;
    pitch *= Math.PI / 180.0;
    yaw *= Math.PI / 180.0;
    double cr = Math.cos(roll * 0.5);
    double sr = Math.sin(roll * 0.5);
    double cp = Math.cos(pitch * 0.5);
    double sp = Math.sin(pitch * 0.5);
    double cy = Math.cos(yaw * 0.5);
    double sy = Math.sin(yaw * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
  }

  @Override
  public void execute() {

    // Get current robot quaternion, if this doesn't work pass RPY into Rot3d and get from that.
    s_Swerve.gyro.get6dQuaternion(temp);
    RPYToQdeg(22.5, 22.5, 60, temp);
    // Quaternion sandwich point tranlsation black magic.
    // Look transforming a vector or point by a quaterinion.
    // We are tranlsating i and j, so we only need to compute the stuff multiplied by the x axis and
    // y axis respectively.
    // We are only really looking at the xy plane so it is not necessary to do any math for the z
    // component of the vectors.
    p1[0] = temp[1] * temp[1] + temp[0] * temp[0] - temp[2] * temp[2] - temp[3] * temp[3];
    p1[1] = 2 * temp[0] * temp[3] + 2 * temp[1] * temp[2];

    p2[0] = (2 * temp[1] * temp[2] - 2 * temp[0] * temp[3]);
    p2[1] = (temp[0] * temp[0] - temp[1] * temp[1] + temp[2] * temp[2] - temp[3] * temp[3]);

    // Assuming normal vector has length of 1, we get the dot product of it with the normal vector
    // of the xy plane.
    // The x and y component of the normal vector of the xy plane are zero.
    // This means that when we do the dot product we only need to find the z component of the
    // robot's normal vector.
    double zcross = p1[0] * p2[1] - p1[1] * p2[0];

    // Make sure the vector is facing the correct direction.
    if (zcross < 0)
      zcross *= -1;
    // No division because magnitudes are 1.
    // Any vector dot product k gives only the Z Component of that vector.
    // Angle formula
    double angle = Math.acos(zcross);
    System.out.println(angle * 180 / Math.PI);
    s_Swerve.drive(new Translation2d(0, 0).times(SwerveConstants.maxSpeed).times(direction), 0,
        true, false);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
