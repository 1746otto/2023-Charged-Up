package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.ArmRollerRunInCommand;
import frc.robot.commands.basic.ArmRollerShootCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;

public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand(ArmPositionSubsystem armPositionSubsystem,
      ArmRollersSubsystem armRollersSubsystem) {
    addCommands(
        new ParallelCommandGroup(new ArmRequestSelectorCommand(armPositionSubsystem, -3000),
            new ArmRollerRunInCommand(armRollersSubsystem)),
        new WaitCommand(0.25), new ArmRollerShootCommand(armRollersSubsystem).withTimeout(.5),
        new ArmRequestSelectorCommand(armPositionSubsystem, ArmConstants.kArmRestPos));
  }
}
