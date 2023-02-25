// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax elevatorMotor;
    private AnalogInput beamBreak;
    private double error;
    private double prevError;
    private double motorSpeed;
    private double speedConstant = ElevatorConstants.kElevatorSpeed;
    private double kP = ElevatorConstants.kElevatorP;
    private double kD = ElevatorConstants.kElevatorD;
    private boolean beamBreakLastState = false;
    private double currState;
    
    public ElevatorSubsystem(){
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor1ID, MotorType.kBrushless);
       // beamBreak = new AnalogInput(ElevatorConstants.kElevatorAnalogInputChannel);
        // elevatorMotor.invert();
        elevatorMotor.getEncoder().setPosition(0);
    }
  
    public void stopElevator(){
        elevatorMotor.stopMotor();
    }
    public boolean beamBreakBroken(){
        return beamBreakLastState;
    }

    public void runToRequest(int reqPosition){
        error = reqPosition - currState;

        motorSpeed = error * kP;
        // motorSpeed = (error - prevError) * kD + (kP * error);

        if (motorSpeed > speedConstant){
            motorSpeed = speedConstant;
        }else if (motorSpeed < -speedConstant){
            motorSpeed = -speedConstant;
        }
        elevatorMotor.set(motorSpeed);

        prevError = error;
    }

    @Override
    public void periodic() {
        //beamBreakLastState = ((Math.floor(beamBreak.getVoltage()) == 0) && (motorSpeed < 0));
        currState = elevatorMotor.getEncoder().getPosition();
        System.out.println("Current State: " + currState);
    }
}
