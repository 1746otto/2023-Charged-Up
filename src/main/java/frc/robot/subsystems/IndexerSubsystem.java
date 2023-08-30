package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IndexerSubsystem {
  private TalonSRX frontSide;

  public IndexerSubsystem() {
    frontSide = new TalonSRX(60);
  }

  public void runFullForward() {
    frontSide.set(ControlMode.PercentOutput, -1.0);
  }

  public void runHalfForward() {
    frontSide.set(ControlMode.PercentOutput, -0.5);
  }

  public void runCustom(double input) {
    frontSide.set(ControlMode.PercentOutput, -input);
  }
}
