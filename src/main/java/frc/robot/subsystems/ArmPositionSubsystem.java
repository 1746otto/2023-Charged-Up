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
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.pathplanner.lib.PathPoint;
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

  private double lastPos;
  private double currPos;

  public ArmPositionSubsystem() {
    encoderConfig = new CANcoderConfiguration();
    armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 130;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.CurrentLimits.StatorCurrentLimit = 130;
    armConfig.Slot0.kP = ArmConstants.kArmP;
    armConfig.Slot0.kD = ArmConstants.kArmD;

    // encoderConfig.MagnetSensor.AbsoluteSensorRange =
    // AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoder.getConfigurator().apply(encoderConfig);
    armMotor.getConfigurator().apply(armConfig);
    armMotor.setInverted(true);
    armEncoder.getPosition().setUpdateFrequency(100);
    armEncoder.getPosition().waitForUpdate(0.1);
    requestPos = ArmConstants.kArmRestPos;
    lastPos = armEncoder.getAbsolutePosition().getValue();
  }

  public void armToRequest(double requestedPosition) {
    // Change position values immediatley for safety reasons
    armMotor.setControl(new PositionDutyCycle(requestedPosition, false, 0.0, 0, true));
    // armMotor.setControl(new DutyCycleOut(0, false, true).withOutput(-0.1));
  }

  public void armStop() {
    // armMotor.set(TalonFXControlMode.PercentOutput, 0);
    armMotor.set(0.0);
  }

  public void disablePID() {
    armConfig.Slot0.kP = 0.0;
    armConfig.Slot0.kP = 0.0;
    armMotor.getConfigurator().apply(armConfig);
  }

  public void enablePID() {
    armConfig.Slot0.kP = ArmConstants.kArmP;
    armConfig.Slot0.kD = ArmConstants.kArmD;
    armMotor.getConfigurator().apply(armConfig);
  }

  public void setHomeSpeed() {
    armMotor.set(0.2);
  }

  public boolean armAtReq(double reqPosition) {
    // return (armEncoder.getPosition() == reqPosition);
    // return ((armEncoder.getAbsolutePosition().getValue()) <= (reqPosition++)
    // && ((armEncoder.getAbsolutePosition().getValue()) >= (reqPosition--)));
    return armEncoder.getAbsolutePosition().getValue() == reqPosition;
  }

  public void zeroEncoder() {
    armEncoder.setPosition(ArmConstants.kArmRestPos);
  }

  public boolean armReqisCorrect(double req) {
    return (requestPos == req);
  }

  public double getRequestedPosition() {
    return armMotor.getPosition().getValue();
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
    SmartDashboard.putNumber("Arm Talon Position: ", armMotor.getPosition().getValue());
    System.out.println("Arm Talon Position: " + (armMotor.getPosition().toString()));

    currPos = armEncoder.getAbsolutePosition().getValue();
    // if (Math.abs(currPos - lastPos) > 0.1) {
    // double relPos = currPos * ArmConstants.CANToIntConvert;
    // requestPos = requestPos + (relPos - requestPos);
    // }
    lastPos = currPos;

    armToRequest(requestPos);
  }
}
