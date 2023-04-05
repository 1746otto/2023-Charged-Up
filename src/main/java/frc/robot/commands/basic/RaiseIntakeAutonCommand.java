package frc.robot.commands.basic;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RaiseIntakeAutonCommand extends ParallelCommandGroup {
  public RaiseIntakeAutonCommand(ArmPositionSubsystem armPosSub, ArmRollersSubsystem armRollerSub) {
    addCommands(new ParallelCommandGroup(new ArmRequestSelectorCommand(armPosSub, -3000),
        new ArmRollerRunInCommand(armRollerSub)));
  }
}
