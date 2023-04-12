package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.ArmRollerOuttakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;

public class LowGoalCommand extends SequentialCommandGroup {
  LowGoalCommand(ArmPositionSubsystem armPositionSubsystem,
      ArmRollersSubsystem armRollersSubsystem) {
    addCommands(
        new ArmRequestSelectorCommand(armPositionSubsystem, ArmConstants.kArmIntakeAndScorePos),
        new ArmRollerOuttakeCommand(armRollersSubsystem).withTimeout(0.5),
        new ArmRequestSelectorCommand(armPositionSubsystem, ArmConstants.kArmRestPos));
  }
}
