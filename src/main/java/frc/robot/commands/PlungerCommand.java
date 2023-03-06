package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.Plungersubsystem;

public class PlungerCommand extends CommandBase{
    private final Plungersubsystem m_Plungersubsystem;

    public PlungerCommand(Plungersubsystem subsystem) {
        m_Plungersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Plungersubsystem.togglePlunger();
        System.out.println(m_Plungersubsystem.isEngaged());
    }
}