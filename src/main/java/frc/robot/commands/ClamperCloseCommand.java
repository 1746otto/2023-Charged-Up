package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClamperCloseCommand extends CommandBase{
    private final ClamperSubsystem m_ClamperSubsystem;

    public ClamperCloseCommand(ClamperSubsystem subsystem) {
        m_ClamperSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_ClamperSubsystem.closeClamper();
        System.out.println("Clamper Closed: " + m_ClamperSubsystem.isEngaged());
    }
}