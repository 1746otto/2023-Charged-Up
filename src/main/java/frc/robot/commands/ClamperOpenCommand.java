package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

// This entire class should be a lambda, or at least an Instant Command.
public class ClamperOpenCommand extends CommandBase{
    private final PlacerSubsystem m_PlacerSubsystem;

    public ClamperOpenCommand(PlacerSubsystem subsystem) {
        m_PlacerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_PlacerSubsystem.openClamper();
    }

    @Override
    public void end(boolean interrupted){
        // Why are we doing this? There is no situation I can think of where this would be required.
        m_PlacerSubsystem.closeClamper();
    }

}