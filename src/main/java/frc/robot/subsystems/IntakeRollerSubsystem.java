// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;;

public class IntakeRollerSubsystem extends SubsystemBase {
  CANSparkMax masterMotor;
  CANSparkMax slaveMotor;

  public IntakeRollerSubsystem() {
    masterMotor = new CANSparkMax(IntakeRollerConstants.CANID1, MotorType.kBrushless);
    slaveMotor = new CANSparkMax(IntakeRollerConstants.CANID2, MotorType.kBrushless);

    slaveMotor.follow(masterMotor, true);
  }

  public void runFullPower() {
    masterMotor.set(IntakeRollerConstants.kFullPower);
  }

  public void runCustomPower(double input) {
    masterMotor.set(input);
  }

  public void runZeroPower() {
    masterMotor.set(0);
  }

  public void runClockwise(double input) {
    masterMotor.set(input);
  }

  public void runCounterClockwise(double input) {
    masterMotor.set((-1 * input));
  }
}
