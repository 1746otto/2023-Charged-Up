package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ConeDunkerSubsytem extends SubsystemBase {
  private CANSparkMax Dunker;
  private double current;

  ConeDunkerSubsytem() {
    Dunker = new CANSparkMax(30, MotorType.kBrushless);
    Dunker.setIdleMode(IdleMode.kBrake);

  }

  public void theDunk() {
    Dunker.set(0.1);
  }

  public void theReverseDunk() {
    Dunker.set(-0.1);
  }

  public void dunkStop() {
    Dunker.stopMotor();
  }

  public boolean currentBroken() {
    return (current >= 50);
  }

  public boolean currentBrokenUp() {
    return (current >= 30);
  }

  @Override
  public void periodic() {
    current = Dunker.getOutputCurrent();
  }
}
