package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.DunkerConstants;

public class ConeDunkerSubsytem extends SubsystemBase {
  private CANSparkMax Dunker;
  private double current;
  private double currentState;

  public ConeDunkerSubsytem() {
    Dunker = new CANSparkMax(DunkerConstants.kCANCoderID, MotorType.kBrushless);
    Dunker.setIdleMode(IdleMode.kBrake);
    Dunker.getEncoder().setPosition(DunkerConstants.KZeroPos);


  }

  public void theDunk() {
    Dunker.set(DunkerConstants.KDunkSpeed);
  }

  public void theReverseDunk() {
    Dunker.set(DunkerConstants.KRDunkSpeed);
  }

  public void dunkStop() {
    Dunker.stopMotor();
  }

  public void setPositionTo0() {
    Dunker.getEncoder().setPosition(DunkerConstants.KZeroPos);
  }

  public boolean DownPosition() {
    return (currentState < DunkerConstants.KDownPosition);
  }

  public boolean UpPosition() {
    return (currentState > DunkerConstants.KUpPosition);
  }

  public boolean currentBroken() {
    return (current >= DunkerConstants.KMaxDownCurrent);
  }

  public boolean currentBrokenUp() {
    return (current >= DunkerConstants.KMaxUpCurrent);
  }

  public double getDunkerEncoderValues() {
    return currentState;
  }

  @Override
  public void periodic() {
    current = Dunker.getOutputCurrent();
    currentState = Dunker.getEncoder().getPosition();
    SmartDashboard.putNumber("Dunker: ", currentState);


  }
}
