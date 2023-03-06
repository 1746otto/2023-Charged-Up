package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FlapToggleCommand extends CommandBase{
    FlapSubsystem m_flapSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;

    public FlapToggleCommand(FlapSubsystem subsystem, ElevatorSubsystem eSubsystem){
        m_flapSubsystem = subsystem;
        m_ElevatorSubsystem = eSubsystem;
        addRequirements(subsystem, eSubsystem);
    }

    @Override
    public void execute(){
        if (m_ElevatorSubsystem.beamBreakBroken()){
            m_flapSubsystem.flapOn();
        }else{
            m_flapSubsystem.flapOff();
        }
    }
}
