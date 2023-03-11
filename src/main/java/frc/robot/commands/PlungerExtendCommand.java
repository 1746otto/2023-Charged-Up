package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;

// Again this could just be a set of lambdas/Instant Commands.
public class PlungerExtendCommand extends CommandBase{
    private final PlacerSubsystem m_Placersubsystem;

    public PlungerExtendCommand(PlacerSubsystem subsystem) {
        m_Placersubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Placersubsystem.extendPlunger();
    }
}