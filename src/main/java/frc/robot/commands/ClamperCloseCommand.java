package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

// This entire class should just be a lambda, or at least an instant command.
public class ClamperCloseCommand extends CommandBase{
    private final PlacerSubsystem m_PlacerSubsystem;
    
    public ClamperCloseCommand(PlacerSubsystem subsystem) {
        m_PlacerSubsystem = subsystem;
        addRequirements(m_PlacerSubsystem);
    }

    @Override
    public void initialize() {
        m_PlacerSubsystem.closeClamper();
        
    }

    @Override
    public boolean isFinished() {
        return !m_PlacerSubsystem.isClamperOpen();
    }
}