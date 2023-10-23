package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.ArmPositionStopCommand;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.ArmRollerShootCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class ShootCubeHighCommand extends SequentialCommandGroup {
  public ShootCubeHighCommand(ElevatorSubsystem elevatorSub, ArmPositionSubsystem armPosSub,
      ArmRollersSubsystem armRollerSub) {
    addCommands(new ParallelCommandGroup(
        new ElevatorRequestSelectorCommand(elevatorSub, ElevatorConstants.kHighCubePosition),
        new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmMidShootPos),
        new WaitCommand(1.5).until(() -> {
          return elevatorSub.getElevatorEncoderValues() > 13.0;
        }).andThen(new ArmRollerShootCommand(armRollerSub).withTimeout(.5))),
        new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmRestPos));
  }
}
