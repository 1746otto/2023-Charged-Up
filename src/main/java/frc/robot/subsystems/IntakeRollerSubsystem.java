// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;;

public class IntakeRollerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax masterMotor;
  CANSparkMax slaveMotor;

  public IntakeRollerSubsystem() {
    masterMotor = new CANSparkMax(IntakeRollerConstants.CANID1, MotorType.kBrushless);
    slaveMotor = new CANSparkMax(IntakeRollerConstants.CANID2, MotorType.kBrushless);
    
    slaveMotor.follow(masterMotor,true);
    

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
