// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeExtensionConstants;

public class IntakeExtensionSubsystem extends SubsystemBase {
  CANSparkMax extensionMotor;
  SparkMaxLimitSwitch bottomLimitSwitch;
  SparkMaxLimitSwitch topLimitSwitch;

  public IntakeExtensionSubsystem() {
    extensionMotor =
        new CANSparkMax(IntakeExtensionConstants.kExtensionMotor, MotorType.kBrushless);
    bottomLimitSwitch =
        extensionMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    topLimitSwitch = extensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extensionMotor.setInverted(true);
  }

  public void setMotorSpeed(double speed) {
    extensionMotor.set(speed);
  }

  public void setMotorExtensionSpeed() {
    setMotorSpeed(IntakeExtensionConstants.kExtensionSpeed);
  }

  public void setMotorRetractionSpeed() {
    setMotorSpeed(IntakeExtensionConstants.kRetractionSpeed);
  }

  public void setMotorStoppedSpeed() {
    setMotorSpeed(IntakeExtensionConstants.kStopSpeed);
  }

  public boolean isExtended() {
    return bottomLimitSwitch.isPressed();
  }

  public boolean isRetracted() {
    return topLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {}

}
