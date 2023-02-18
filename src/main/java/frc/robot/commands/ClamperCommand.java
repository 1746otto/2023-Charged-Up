package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamper;

public class ClamperCommand extends CommandBase{
    private final Clamper m_placer2;
    public ClamperCommand(Clamper subsystem) {
        m_placer2 = subsystem;
        addRequirements(subsystem);
    }
    private void addRequirements(Clamper subsystem) {
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