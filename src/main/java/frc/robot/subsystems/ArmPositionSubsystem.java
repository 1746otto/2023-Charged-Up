package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ArmConstants;

public class ArmPositionSubsystem extends SubsystemBase {
  // TODO: Change data types to correct types
  private TalonFX armMotor;
  private double armEncoder;

  public ArmPositionSubsystem() {
    armMotor = new TalonFX(ArmConstants.kArmPosMotorID);
    armMotor.configStatorCurrentLimit(null);
  }

  public void armToIntakeOrScore() {
    armMotor.set(TalonFXControlMode.Position, ArmConstants.kArmIntakeAndScorePos);
  }

  public void armToRest() {
    armMotor.set(TalonFXControlMode.Position, ArmConstants.kArmRestPos);
  }

  public double getArmEncoderValues() {
    return armEncoder;
  }

  @Override
  public void periodic() {
    armEncoder = armMotor.getSelectedSensorPosition();
  }
}
