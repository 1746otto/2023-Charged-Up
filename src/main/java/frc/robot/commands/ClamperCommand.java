package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClamperSubsystem;

public class ClamperCommand extends CommandBase{
    private final ClamperSubsystem m_placer2;
    public ClamperCommand(ClamperSubsystem subsystem) {
        m_placer2 = subsystem;
        addRequirements(subsystem);
    }
    private void addRequirements(ClamperSubsystem subsystem) {
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