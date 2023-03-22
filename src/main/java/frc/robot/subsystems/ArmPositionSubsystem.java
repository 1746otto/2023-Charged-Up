package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.math.Conversions;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class ArmPositionSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private WPI_CANCoder armEncoder;
  private BaseTalonPIDSetConfiguration armPIDController;

  public ArmPositionSubsystem() {
    armMotor = new TalonFX(ArmConstants.kArmPosMotorID);
    armPIDController = new BaseTalonPIDSetConfiguration(FeedbackDevice.Analog);
    armMotor.config_kP(0, 0);
    armMotor.config_kD(0, 0);
    // armMotor.configMotionAcceleration(1000);
    armEncoder = new WPI_CANCoder(ArmConstants.kCANCoderID);
    // armEncoder.setPosition(0);
  }

  public void armToCustom(double reqPosition) {
    armMotor.set(TalonFXControlMode.Position,
        Conversions.degreesToFalcon(Conversions
            .CANcoderToDegrees(ArmConstants.kArmIntakeAndScorePos, ArmConstants.kArmGearRatio),
            ArmConstants.kArmGearRatio));
  }

  public void armStop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    System.out.println(armEncoder.getPosition());
  }
}
