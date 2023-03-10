package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

public class PlungerCommand extends CommandBase{
    private final PlacerSubsystem m_PlacerSubsystem;
    public PlungerCommand(PlacerSubsystem subsystem) {
        m_PlacerSubsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        m_PlacerSubsystem.extendPlunger();
    }
    @Override
    public void end(boolean interrupted) {
        m_PlacerSubsystem.retractPlunger();
    }
}