package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveOverChargeStationCommand extends CommandBase {
  private Swerve swerve;
  private int direction = 1;
  private double[] p1;
  private double[] p2;
  private double zcross;
  private double[] temp;
  private int stage;

  public DriveOverChargeStationCommand(Swerve swerve) {
    this.swerve = swerve;

    p1 = new double[2];
    p2 = new double[2];
    temp = new double[4];
    stage = 0;

    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Red)
      direction = -1;
  }

  @Override
  public void execute() {
    swerve.gyro.get6dQuaternion(temp);
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
    zcross = p1[0] * p2[1] - p1[1] * p2[0];

    // Make sure the vector is facing the correct direction.
    if (zcross < 0)
      zcross *= -1;

    // No division because magnitudes are 1.
    // Any vector dot product k gives only the Z Component of that vector.
    // Angle formula
    double angle = Math.acos(zcross);
    // Why might I do this you ask, well, because I can and it's cool.

    if (stage == 0) {
      if (angle > Math.PI / 15.0) {
        stage = 1;
      }
      swerve.drive(new Translation2d(0.6 * direction, 0.0), 0.0, true, false);
    }

  }

}
