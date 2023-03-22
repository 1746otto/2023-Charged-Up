package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ArmConstants;

public class ArmPositionSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private double armEncoder;

  public ArmPositionSubsystem() {
    armMotor = new TalonFX(ArmConstants.kArmPosMotorID);
    armMotor.configStatorCurrentLimit(null);
  }

  public void armToCustom(double reqPosition) {
    armMotor.set(TalonFXControlMode.Position, reqPosition);
  }

  public void armStop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    armEncoder = armMotor.getSelectedSensorPosition();
    System.out.println(armEncoder);
  }
}
