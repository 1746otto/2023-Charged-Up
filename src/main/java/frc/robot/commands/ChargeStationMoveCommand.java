package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class ChargeStationMoveCommand extends CommandBase {
  private Swerve swerve;
  private int direction = 1;
  private double[] p1;
  private double[] p2;
  private double zcross;
  private double[] temp;
  private int stage;
  private double xpos = 0.0;
  private BuiltInAccelerometer accelerometer;
  private Timer timer;
  private double xError;
  private double deltaError;
  private double prevError;
  private double speed;
  private double kP;
  private double kD;


  public ChargeStationMoveCommand(Swerve swerve) {
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
    xError = 0;
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

    // Not a real angle but it is good enough.
    double accelMag = Math.sqrt(
        accelerometer.getX() * accelerometer.getX() + accelerometer.getY() * accelerometer.getY()
            + accelerometer.getZ() * accelerometer.getZ());

    switch (stage) {
      case 0:
        if (angle > Math.toRadians(15))
          stage = 1;
        swerve.drive(new Translation2d(3.5 * direction, 0), 0, true, false);
      case 1:
        timer.start();
        swerve.drive(new Translation2d(1.5, 0), 0, true, false);
        stage = 2;
      case 2:
        // Accel mag is in gs and idk what a negligable amount of gs is
        if (angle > Math.toRadians(1) || accelMag > .05)
          timer.reset();
        if (timer.get() > 0.5) {
          stage = 3;
          swerve.drive(new Translation2d(3.5 * direction * -1, 0), 0, true, false);
        }
      case 3:
        if (angle > Math.toRadians(15)) {
          // May need to change direction here
          stage = 4;
        }
      case 4:
        double[] velocities = new double[3];
        swerve.gyro.getRawGyro(velocities);
        if (swerve.gyro.getRoll() != 0) {
          xError = -(swerve.gyro.getRoll());
          deltaError = xError - prevError;
          System.out.println("Roll: " + xError);

          speed = kP * xError + (-kD) * velocities[0];

          if (speed > SwerveConstants.autonDriveSpeed) {
            speed = SwerveConstants.autonDriveSpeed;
          } else if (speed < -SwerveConstants.autonDriveSpeed) {
            speed = -SwerveConstants.autonDriveSpeed;
          }
          prevError = xError;

        }

        swerve.drive(
            new Translation2d(speed * direction, 0).times(SwerveConstants.maxSpeed).times(1), 0.0,
            true, false);
    }

  }

  @Override
  public boolean isFinished() {
    return stage > 3;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0.0, true, false);
  }
}
