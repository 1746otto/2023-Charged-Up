package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ArmPositionSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.commands.basic.ArmRequestSelectorCommand;

public class IntakeCubeAutonCommand extends SequentialCommandGroup {
  IntakeCubeAutonCommand(ArmPositionSubsystem m_armPositionSubsystem,
      ArmRollersSubsystem m_armRollersSubsystem) {
    addCommands(
        new ParallelDeadlineGroup(new ArmRollerIntakeCommand(m_armRollersSubsystem),
            new ArmRequestSelectorCommand(m_armPositionSubsystem,
                ArmConstants.kArmIntakeAndScorePos)),
        new ArmRequestSelectorCommand(m_armPositionSubsystem, ArmConstants.kArmRestPos));

  }
}
