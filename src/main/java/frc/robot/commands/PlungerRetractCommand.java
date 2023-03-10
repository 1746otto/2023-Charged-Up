package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;
import java.util.function.DoubleSupplier;

// Again this could just be a set of lambdas/Instant Commands.
// We don't need this bloat in our code base.
// For those that were here last year, remember how the code looked?
// And how everything was a pain to use because it was all spaghetti.
public class PlungerRetractCommand extends CommandBase{
    private final PlacerSubsystem m_Placersubsystem;

    public PlungerRetractCommand(PlacerSubsystem subsystem) {
        m_Placersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Placersubsystem.retractPlunger();
    }
}