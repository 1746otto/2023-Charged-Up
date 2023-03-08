package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.PlungerSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PlungerExtendCommand extends CommandBase{
    private final PlungerSubsystem m_Plungersubsystem;

    public PlungerExtendCommand(PlungerSubsystem subsystem, DoubleSupplier elevatorEncoder) {
        m_Plungersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Plungersubsystem.extendPlunger();
        System.out.println("Plunger Extended: " + m_Plungersubsystem.isEngaged());
    }
}