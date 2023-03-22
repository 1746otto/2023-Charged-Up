package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;

public class ArmPositionSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private double armEncoder;
  private BaseTalonPIDSetConfiguration armPIDController;

  public ArmPositionSubsystem() {
    armMotor = new TalonFX(ArmConstants.kArmPosMotorID);
    armMotor.configStatorCurrentLimit(null);
    armPIDController = new BaseTalonPIDSetConfiguration(FeedbackDevice.Analog);
    armMotor.config_kP(0, ArmConstants.kArmP);
    armMotor.configMotionAcceleration(armEncoder);
    armMotor.config_kD(0, 0);
    armMotor.config_kI(0, 0);
    armMotor.config_kF(0, 0);
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
