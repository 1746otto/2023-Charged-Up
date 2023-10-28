package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmPositionSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.commands.basic.ArmRequestSelectorCommand;

public class IntakeCubeAutonCommand extends SequentialCommandGroup {
  public IntakeCubeAutonCommand(ElevatorSubsystem elevatorSub, ArmPositionSubsystem armPosSub,
      ArmRollersSubsystem armRollerSub) {
    addCommands(
        new ParallelCommandGroup(
            new ElevatorRequestSelectorCommand(elevatorSub,
                ElevatorConstants.kCubeElevatorIntakePos),
            new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmIntakeAndScorePos)),
        new WaitCommand(0.5), new ArmRollerIntakeCommand(armRollerSub).withTimeout(1),
        new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmRestPos));

  }
}

