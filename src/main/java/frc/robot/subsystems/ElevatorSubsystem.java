// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class ElevatorSubsystem extends SubsystemBase{
    private static final Mode AnalogMode = null;
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private AnalogInput beamBreak;
    private SparkMaxLimitSwitch limitSwitch;
    private double error;
    private double prevError;
    private double motorSpeed;
    private double speedConstant = ElevatorConstants.kElevatorSpeed;
    private double kP = ElevatorConstants.kElevatorP;
    private double kD = ElevatorConstants.kElevatorD;
    private boolean beamBreakLastState;
    private double currState;
    
    public ElevatorSubsystem(){
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor1ID, MotorType.kBrushless);
        beamBreak = new AnalogInput(ElevatorConstants.kElevatorAnalogInputChannel);
        // elevatorMotor.invert();
        limitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        pidController = elevatorMotor.getPIDController();
        pidController.setP(kP, 0);
        // pidController.setD(kD, 0);
        // pidController.setFF(.005, 0);
        pidController.setOutputRange(-1, 1);
        pidController.setSmartMotionMaxVelocity(speedConstant, 0);
        pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    }
  
    public void stopElevator(){
        elevatorMotor.stopMotor();
    }
    public boolean beamBreakBroken(){
        return beamBreakLastState;
    }
    public boolean limitSwitchActivated(){
        return limitSwitch.isPressed();
    }
    public void elevatorRunDown(){
        elevatorMotor.set(-0.5);
    }
    public void setPositionTo0(){
        elevatorMotor.getEncoder().setPosition(0);
    }
    public double getElevatorEncoderValues(){
        return currState;
    }

    public void runToRequest(int reqPosition){
        // error = Math.abs(reqPosition - currState);

        // motorSpeed = error * kP;
        // motorSpeed = (error - prevError) * kD + (kP * error);

        // if (motorSpeed > speedConstant){
        //     motorSpeed = speedConstant;
        // }else if (motorSpeed < -speedConstant){
        //     motorSpeed = -speedConstant;
        // }

        // if (motorSpeed > speedConstant){
        //     motorSpeed = Math.signum(reqPosition-currState) * speedConstant;
        // }

        // elevatorMotor.set(motorSpeed);

        // prevError = error;
        pidController.setReference(reqPosition, ControlType.kSmartMotion);
    }

    @Override
    public void periodic() {
        beamBreakLastState = ((Math.round(beamBreak.getVoltage()) == 0) && (elevatorMotor.get() < 0));
        currState = elevatorMotor.getEncoder().getPosition();
        //System.out.println("Current State: " + currState);
    }
}