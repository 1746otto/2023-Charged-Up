package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonGyroReset extends CommandBase {
  double initialAngle; // Angle it is supposed to start at.
  double startAngle; // Angle it actually starts at.
  DoubleSupplier gyroAngle;
  Consumer<Double> gyroAngleSetter;

  public AutonGyroReset(double realIntialAngle, DoubleSupplier gyroAngle,
      Consumer<Double> gyroAngleSetter) {
    initialAngle = realIntialAngle;
    this.gyroAngle = gyroAngle;
    this.gyroAngleSetter = gyroAngleSetter;
  }

  @Override
  public void initialize() {
    startAngle = gyroAngle.getAsDouble();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("initial angle", initialAngle);
    SmartDashboard.putNumber("gyro angle", gyroAngle.getAsDouble());
    SmartDashboard.putNumber("accepted angle", gyroAngle.getAsDouble() - startAngle + initialAngle);
  }

  @Override
  public void end(boolean interrupted) {
    gyroAngleSetter.accept(gyroAngle.getAsDouble() - startAngle + initialAngle);
  }
}
