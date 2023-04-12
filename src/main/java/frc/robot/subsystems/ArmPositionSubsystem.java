package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPositionSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private CANCoder armEncoder;
  private BaseTalonPIDSetConfiguration armPIDController;
  private double requestPos = 0;

  public ArmPositionSubsystem() {
    armMotor = new TalonFX(ArmConstants.kArmPosMotorID);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armEncoder = new CANCoder(ArmConstants.kCANCoderID);
    armMotor.setSelectedSensorPosition(0.0);
    armPIDController = new BaseTalonPIDSetConfiguration(FeedbackDevice.Analog);
    armMotor.config_kP(0, ArmConstants.kArmP);
    armMotor.config_kD(0, 0);
    armMotor.configClosedLoopPeakOutput(0, 0.2, 0);
    requestPos = ArmConstants.kArmRestPos;
    // armMotor.configMotionAcceleration(1000)
    armMotor.setInverted(true);

  }

  public double getRequestedPosition() {
    return requestPos;
  }

  public void zeroEncoder() {
    armMotor.setSelectedSensorPosition(600);
  }

  public void disablePID() {
    armMotor.configClosedLoopPeakOutput(0, 0, 0);
  }

  public void enablePID() {
    armMotor.configClosedLoopPeakOutput(0, 0.2, 0);
  }

  public void armToRequest(double requestedPosition) {
    armMotor.set(TalonFXControlMode.Position, requestedPosition);
  }

  public void setHomeSpeed() {
    armMotor.set(TalonFXControlMode.PercentOutput, .2);
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
    SmartDashboard.putNumber("CANCoder: ", armEncoder.getAbsolutePosition());
    // System.out.println("Relative Encoder: " + armMotor.getSelectedSensorPosition());
    // armMotor.setSelectedSensorPosition(armEncoder.getAbsolutePosition()
    // * (ArmConstants.kArmGearRatio * ArmConstants.kCANTickToFalConversion)); // cancoder: 4096
    // // Falcon: 20
    SmartDashboard.putNumber("Arm Encoder: ", armMotor.getSelectedSensorPosition());
    armToRequest(requestPos);
    SmartDashboard.putNumber("Target Position", getRequestedPosition());

  }
}
