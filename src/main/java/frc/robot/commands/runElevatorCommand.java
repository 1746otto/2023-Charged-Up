package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class runElevatorCommand extends CommandBase{
    private final ElevatorSubsystem m_elevator;
    public runElevatorCommand(ElevatorSubsystem subsystem){
        m_elevator = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevator.runElevatorUp();
    }
    @Override
    public void end(boolean isFinished){
        m_elevator.stopElevator();
    }
}
