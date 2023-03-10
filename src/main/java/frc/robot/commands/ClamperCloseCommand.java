package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

// This entire class should just be a lambda, or at least an instant command.
public class ClamperCloseCommand extends CommandBase{
    private final PlacerSubsystem m_PlacerSubsystem;
    private BooleanSupplier indexerBeamBreak;
    private BooleanSupplier elevatorBeamBreak;
    
    public ClamperCloseCommand(PlacerSubsystem subsystem) {
        m_PlacerSubsystem = subsystem;
        addRequirements(m_PlacerSubsystem);
    }

    @Override
    public void initialize() {
            m_PlacerSubsystem.closeClamper();
    }
}