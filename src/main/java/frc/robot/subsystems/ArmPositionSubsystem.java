package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.module.InvalidModuleDescriptorException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.math.Conversions;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycle;
// import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
// import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// New stuff
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

public class ArmPositionSubsystem extends SubsystemBase {
  // private TalonFX armMotor;
  // private CANCoder armEncoder;
  // private BaseTalonPIDSetConfiguration armPIDController;
  // Arm Motor
  private DeviceIdentifier armId;
  private DeviceIdentifier enId;
  private TalonFXConfiguration armConfig;
  // private TalonFX armMotor;
  // CANCoder
  private CANcoderConfiguration encoderConfig;
  // private CANcoder armEncoder;
  private double requestPos;

  private CANcoder armEncoder = new CANcoder(ArmConstants.kCANCoderID, "rio");
  private TalonFX armMotor = new TalonFX(ArmConstants.kArmPosMotorID, "rio");

  public ArmPositionSubsystem() {
    encoderConfig = new CANcoderConfiguration();
    armConfig = new TalonFXConfiguration();
    encoderConfig.serialize();
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 130;
    armEncoder.getConfigurator().apply(encoderConfig);
    armMotor.getConfigurator().apply(armConfig);
    armConfig.Slot0.kP = ArmConstants.kArmP;
    armMotor.setInverted(true);
    armEncoder.getPosition().setUpdateFrequency(100);
    armEncoder.getPosition().waitForUpdate(0.1);
  }

  public void armToRequest(double requestedPosition) {
    // armMotor.set(TalonFXControlMode.Position, requestedPosition);
    // armMotor.setControl(new PositionDutyCycle(requestedPosition));

    // Change position values immediatley for safety reasons
    armMotor.setControl(new PositionDutyCycle(0.0, false, 0.0, 0, true).withPosition(-2.5));
    // armMotor.setControl(new DutyCycleOut(0, false, true).withOutput(-0.1));
  }

  public void armStop() {
    // armMotor.set(TalonFXControlMode.PercentOutput, 0);
    armMotor.set(0.0);
  }

  public boolean armAtReq(double reqPosition) {
    // return (armEncoder.getPosition() == reqPosition);
    // return ((armEncoder.getAbsolutePosition().getValue()) <= (reqPosition++)
    // && ((armEncoder.getAbsolutePosition().getValue()) >= (reqPosition--)));
    return armEncoder.getAbsolutePosition().getValue() == reqPosition;
  }

  public boolean armReqisCorrect(double req) {
    return (requestPos == req);
  }

  public void setRequest(double request) {
    requestPos = request;
  }

  public void armRun() {
    // armMotor.set(TalonFXControlMode.PercentOutput, 0.1);
    armMotor.set(0.1);
  }

  @Override
  public void periodic() {
    // System.out.println("CANCoder: " + armEncoder.getPosition());
    // System.out.println("Relative Encoder: " + armMotor.getSelectedSensorPosition());
    // armMotor.setSelectedSensorPosition(armEncoder.getAbsolutePosition()
    // * (ArmConstants.kArmGearRatio * ArmConstants.kCANTickToFalConversion)); // cancoder: 4096
    // // Falcon: 20
    SmartDashboard.putNumber("Arm CANCoder: ", (armEncoder.getAbsolutePosition().getValue()));
    System.out.println("Arm CANCoder: " + (armEncoder.getAbsolutePosition().toString()));
    System.out.println("CANCoder relPos: " + armEncoder.getPosition().getValue());
    SmartDashboard.putNumber("Arm Talon Position: ", armMotor.getPosition().getValue());
    System.out.println("Arm Talon Position: " + (armMotor.getPosition().toString()));

    armToRequest(requestPos);
  }
}
