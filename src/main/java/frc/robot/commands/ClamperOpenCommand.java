package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import java.util.function.DoubleSupplier;

public class ClamperOpenCommand extends CommandBase{
    private final ClamperSubsystem m_ClamperSubsystem;

    public ClamperOpenCommand(ClamperSubsystem subsystem) {
        m_ClamperSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_ClamperSubsystem.openClamper();
        System.out.println("Clamper Opened: " + m_ClamperSubsystem.isEngaged());
    }
    @Override
    public void end(boolean interrupted){
        m_ClamperSubsystem.closeClamper();
    }

}