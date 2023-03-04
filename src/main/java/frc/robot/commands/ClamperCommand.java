package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;

public class ClamperCommand extends CommandBase{
    private final ClamperSubsystem m_ClamperSubsystem;

    public ClamperCommand(ClamperSubsystem subsystem) {
        m_ClamperSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_ClamperSubsystem.toggleClamper();
        System.out.println(m_ClamperSubsystem.isEngaged());
    }
}