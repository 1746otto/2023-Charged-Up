package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerVerticalSubsystem;

public class PlacerVerticalCommand extends CommandBase{
    private final PlacerVerticalSubsystem m_placer;
    public PlacerVerticalCommand(PlacerVerticalSubsystem subsystem) {
        m_placer = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        m_placer.extendPlacerUp();
    }
    @Override
    public void end(boolean interrupted) {
        m_placer.DisengagePlacer();
    }
    @Override
    public boolean isFinished() {
        return !m_placer.isAtTop();
    }
}