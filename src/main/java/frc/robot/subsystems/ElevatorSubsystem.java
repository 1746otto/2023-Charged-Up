// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private final double kP = ElevatorConstants.kElevatorP;
  // private final double kD = ElevatorConstants.kElevatorD;

  private CANSparkMax elevatorMotor;
  private SparkMaxPIDController pidController;
  private AnalogInput beamBreak;
  private SparkMaxLimitSwitch limitSwitch;
  private boolean beamBreakLastState;
  private double currState;
  private double reqPosition;

  // Make a vacation home for the elevator
  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor1ID, MotorType.kBrushless);
    beamBreak = new AnalogInput(ElevatorConstants.kElevatorAnalogInputChannel);
    limitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    pidController = elevatorMotor.getPIDController();
    pidController.setP(kP, 0);
    // pidController.setD(kD, 0);
    // pidController.setFF(.005, 0);
    pidController.setOutputRange(-0.3, 0.3);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.getEncoder().setPosition(0);
    reqPosition = ElevatorConstants.kOriginPosition;
    // pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // pidController.setReference(0, ControlType.kSmartMotion, 0);
  }

  public void elevatorRunUp() {
    elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  public void elevatorRunDown() {
    elevatorMotor.set(-0.1);
  }

  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public boolean isElevatorAtReq(double reqPosition) {
    return (getElevatorEncoderValues() == reqPosition);
  }

  public boolean beamBreakBroken() {
    return beamBreakLastState;
  }

  public boolean limitSwitchActivated() {
    return limitSwitch.isPressed();
  }

  public void setPositionTo0() {
    elevatorMotor.getEncoder().setPosition(0);
  }

  public double getElevatorEncoderValues() {
    return currState;
  }

  public void setElevatorReq(double req) {
    reqPosition = req;
  }

  public boolean reqIsCorrect(double req) {
    return (reqPosition == req);
  }

  public void runToRequest(double requestPos) {
    if (limitSwitchActivated() || beamBreakBroken()) {
      stopElevator();
      if (beamBreakBroken()) {
        setPositionTo0();
      }
    } else {
      pidController.setReference(reqPosition, ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    currState = elevatorMotor.getEncoder().getPosition();
    beamBreakLastState = ((Math.floor(beamBreak.getVoltage()) > 0) && (elevatorMotor.get() < 0));
    runToRequest(reqPosition);
    SmartDashboard.putNumber("Elevator: ", currState);
    SmartDashboard.putBoolean("Beambreak: ", beamBreakBroken());
    SmartDashboard.putBoolean("LimitSwitch: ", limitSwitchActivated());
  }
}
