package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacerHorizontalSubsystem;

public class PlacerHorizontalCommand extends CommandBase{
    private final PlacerHorizontalSubsystem m_placer2;
    public PlacerHorizontalCommand(PlacerHorizontalSubsystem subsystem) {
        m_placer2 = subsystem;
        addRequirements(subsystem);
    }
    private void addRequirements(PlacerHorizontalSubsystem subsystem) {
    }
    @Override
    public void initialize() {
        m_placer2.extendPlacerIn();
    }
    @Override
    public void end(boolean interrupted) {
        m_placer2.DisengagePlacer();
    }
    @Override
    public boolean isFinished() {
        return !m_placer2.isAtSide();
    }
}