package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.ArmRollerRunInCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;

public class AutonExtendIntakeCommand extends ParallelCommandGroup {

  public AutonExtendIntakeCommand(ArmPositionSubsystem armPosSub,
      ArmRollersSubsystem armRollerSub) {
    addCommands(new ParallelCommandGroup(
        new ArmRequestSelectorCommand(armPosSub, -ArmConstants.kArmBowlPos),
        new ArmRollerRunInCommand(armRollerSub)));
  }
}
