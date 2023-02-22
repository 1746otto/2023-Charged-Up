package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;
import frc.robot.subsystems.Plungersubsystem;

public class PlungerCommand extends CommandBase{
    private final Plungersubsystem m_PlungerSubsystem;
    public PlungerCommand(Plungersubsystem subsystem) {
        m_PlungerSubsystem = subsystem;
        addRequirements(subsystem);
    }
    private void addRequirements(ClamperSubsystem subsystem) {
    }
    @Override
    public void initialize() {
        m_PlungerSubsystem.extendPlacerDown();
    }
    @Override
    public void end(boolean interrupted) {
        m_PlungerSubsystem.DisengagePlacer();
    }
    @Override
    public boolean isFinished() {
        return !m_PlungerSubsystem.isAtSide();
    }
}