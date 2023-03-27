package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.commands.basic.ArmRunToRequestCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;

public class IntakingSequentialCommand extends SequentialCommandGroup {
  public IntakingSequentialCommand(ElevatorSubsystem elevatorSubsystem,
      ArmRollersSubsystem armRollersSubsystem, ArmPositionSubsystem armPositionSubsystem) {
    addCommands(new ParallelDeadlineGroup(new ArmRollerIntakeCommand(armRollersSubsystem),
        new ElevatorRunToRequestCommand(elevatorSubsystem, ElevatorConstants.kConeIntakePos),
        new SequentialCommandGroup(
            new WaitCommand(0.4)
                .until(() -> elevatorSubsystem.isElevatorAtReq(ElevatorConstants.kConeIntakePos)),
            new ArmRunToRequestCommand(armPositionSubsystem, ArmConstants.kArmIntakeAndScorePos))),
        new ParallelDeadlineGroup(new SequentialCommandGroup(
            new WaitCommand(0.2)
                .until(() -> armPositionSubsystem.armAtReq(ArmConstants.kArmRestPos)),
            new ElevatorRunToRequestCommand(elevatorSubsystem, ElevatorConstants.kOriginPosition)
                .until(() -> elevatorSubsystem.isElevatorAtReq(ElevatorConstants.kOriginPosition))),
            new ArmRunToRequestCommand(armPositionSubsystem, ArmConstants.kArmRestPos)));
  }
}