package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;

public class ClamperCommand extends CommandBase{
    private final ClamperSubsystem m_ClamperSubsystem;
    public ClamperCommand(ClamperSubsystem subsystem) {
        m_ClamperSubsystem = subsystem;
        addRequirements(subsystem);
    }
    private void addRequirements(ClamperSubsystem subsystem) {
    }
    @Override
    public void initialize() {
        m_ClamperSubsystem.extendPlacerIn();
    }
    @Override
    public void end(boolean interrupted) {
        m_ClamperSubsystem.DisengagePlacer();
    }
    @Override
    public boolean isFinished() {
        return !m_ClamperSubsystem.isAtSide();
    }
}