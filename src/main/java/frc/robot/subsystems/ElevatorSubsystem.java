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
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class ElevatorSubsystem extends SubsystemBase {
  private final double kSpeedConstant = ElevatorConstants.kElevatorSpeed;
  private final double kP = ElevatorConstants.kElevatorP;
  // private final double kD = ElevatorConstants.kElevatorD;

  private CANSparkMax elevatorMotor;
  private SparkMaxPIDController pidController;
  private AnalogInput beamBreak;
  private SparkMaxLimitSwitch limitSwitch;
  private boolean beamBreakLastState;
  private double currState;

  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor1ID, MotorType.kBrushless);
    beamBreak = new AnalogInput(ElevatorConstants.kElevatorAnalogInputChannel);
    limitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    pidController = elevatorMotor.getPIDController();
    pidController.setP(kP, 0);
    // pidController.setD(kD, 0);
    // pidController.setFF(.005, 0);
    pidController.setOutputRange(-1, 1);
    pidController.setSmartMotionMaxVelocity(kSpeedConstant, 0);
    pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
  }

  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public boolean beamBreakBroken() {
    return beamBreakLastState;
  }

  public boolean limitSwitchActivated() {
    return limitSwitch.isPressed();
  }

  public void elevatorRunDown() {
    elevatorMotor.set(-0.1);
  }

  public void setPositionTo0() {
    elevatorMotor.getEncoder().setPosition(0);
  }

  public double getElevatorEncoderValues() {
    return currState;
  }

  public void runToRequest(int reqPosition) {
    pidController.setReference(reqPosition, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    beamBreakLastState = (Math.floor(beamBreak.getVoltage()) > 0 && (elevatorMotor.get() < 0));
    // System.out.println("Beam break: " + beamBreakLastState);
    currState = elevatorMotor.getEncoder().getPosition();
  }
}
