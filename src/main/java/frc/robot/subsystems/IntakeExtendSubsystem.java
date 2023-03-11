// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeExtendConstants;

public class IntakeExtendSubsystem extends SubsystemBase {
  CANSparkMax masterMotor;
  SparkMaxLimitSwitch limitSwitch1;
  SparkMaxLimitSwitch limitSwitch2;


  public IntakeExtendSubsystem() {
    masterMotor = new CANSparkMax(IntakeExtendConstants.CANID1, MotorType.kBrushless);
    limitSwitch1 = masterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    limitSwitch2 = masterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    masterMotor.setInverted(true);
  }

  public void extend() {
    masterMotor.set(IntakeExtendConstants.kFullPower);
  }

  public void retract() {
    masterMotor.set(-IntakeExtendConstants.kFullPower);
  }

  public void stopRunning() {
    masterMotor.set(IntakeExtendConstants.kZeroPower);

  }

  public boolean isExtended() {
    return limitSwitch1.isPressed();
  }

  public boolean isRetracted() {
    return limitSwitch2.isPressed();
  }
}
