package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerHorizontalSubsystem;

public class PlacerHorizontalCommand {
    private final PlacerHorizontalSubsystem m_placer;
    public PlacerHorizontalCommand(PlacerHorizontalSubsystem subsystem) {
        m_placer = subsystem;
        addRequirements(subsystem);
       
    }
}