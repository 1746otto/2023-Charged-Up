package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// Again this could just be a set of lambdas/Instant Commands.
public class PlungerExtendCommand extends CommandBase{
    private final PlacerSubsystem m_Placersubsystem;

    public PlungerExtendCommand(PlacerSubsystem subsystem, DoubleSupplier elevatorEncoderValues) {
        m_Placersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Placersubsystem.extendPlunger();
    }
}