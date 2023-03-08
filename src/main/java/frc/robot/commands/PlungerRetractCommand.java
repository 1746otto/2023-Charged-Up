package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.PlungerSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PlungerRetractCommand extends CommandBase{
    private final PlungerSubsystem m_Plungersubsystem;

    public PlungerRetractCommand(PlungerSubsystem subsystem) {
        m_Plungersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Plungersubsystem.retractPlunger();
        System.out.println("Plunger Retracted: " + m_Plungersubsystem.isEngaged());
    }
}