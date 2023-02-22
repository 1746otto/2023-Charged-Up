// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeExtendConstants;

public class IntakeExtendSubsystem extends SubsystemBase {
/** Creates a new ExampleSubsystem. */
  CANSparkMax masterMotor;
  CANSparkMax slaveMotor;
  DigitalInput limitSwitch1;
  DigitalInput limitSwitch2;


  public IntakeExtendSubsystem() {
    masterMotor = new CANSparkMax(IntakeExtendConstants.CANID1, MotorType.kBrushless);
    slaveMotor = new CANSparkMax(IntakeExtendConstants.CANID2, MotorType.kBrushless);
    limitSwitch1 = new DigitalInput(IntakeExtendConstants.kLimitSwitch1);
    limitSwitch2 = new DigitalInput(IntakeExtendConstants.kLimitSwitch2);

    slaveMotor.follow(masterMotor);
    slaveMotor.setInverted(true);
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
    return limitSwitch1.get();
  }

  public boolean isRetracted() {
    return limitSwitch2.get();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}