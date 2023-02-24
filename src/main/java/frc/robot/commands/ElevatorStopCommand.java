// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStopCommand extends CommandBase{
    private final ElevatorSubsystem m_elevator;

    public ElevatorStopCommand(ElevatorSubsystem subsystem){
        m_elevator = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.stopElevator();
    }
}
