package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.math.Conversions;
import frc.robot.constants.ArmConstants;
// import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
// import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// New stuff
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

public class ArmPositionSubsystem extends SubsystemBase {
  // private TalonFX armMotor;
  // private CANCoder armEncoder;
  // private BaseTalonPIDSetConfiguration armPIDController;
  // Arm Motor
  private TalonFXConfigurator armMotor;
  private DeviceIdentifier id = new DeviceIdentifier(ArmConstants.kArmPosMotorID, "TalonFX", "123");
  private TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private TalonFX armActRequest = new TalonFX(ArmConstants.kArmPosMotorID);
  // CANCoder
  private CANcoderConfigurator armEncoder;
  private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
  private CANcoder encoderRequest = new CANcoder(ArmConstants.kCANCoderID);
  private double requestPos;

  public ArmPositionSubsystem() {
    armMotor = new TalonFXConfigurator(id);
    armEncoder = new CANcoderConfigurator(id);
    armMotor.apply(armConfig);
    armEncoder.apply(encoderConfig);
  }

  public void armToRequest(double requestedPosition) {
    armMotor.set(TalonFXControlMode.Position, requestedPosition);
  }

  public void armStop() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public boolean armAtReq(double reqPosition) {
    return (armEncoder.getPosition() == reqPosition);
  }

  public boolean armReqisCorrect(double req) {
    return (requestPos == req);
  }

  public void setRequest(double request) {
    requestPos = request;
  }

  public void armRun() {
    armMotor.set(TalonFXControlMode.PercentOutput, 0.1);
  }

  @Override
  public void periodic() {
    // System.out.println("CANCoder: " + armEncoder.getPosition());
    // System.out.println("Relative Encoder: " + armMotor.getSelectedSensorPosition());
    // armMotor.setSelectedSensorPosition(armEncoder.getAbsolutePosition()
    // * (ArmConstants.kArmGearRatio * ArmConstants.kCANTickToFalConversion)); // cancoder: 4096
    // // Falcon: 20
    SmartDashboard.putNumber("Arm Encoder: ", armMotor.getSelectedSensorPosition());
    armToRequest(requestPos);
  }
}
