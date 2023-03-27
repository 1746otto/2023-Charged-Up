package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.commands.basic.ArmRollerOuttakeCommand;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;

public class OuttakingSequentialCommand extends SequentialCommandGroup {
  public OuttakingSequentialCommand(double elevatorPose, ElevatorSubsystem elevatorSubsystem,
      ArmRollersSubsystem armRollersSubsystem, ArmPositionSubsystem armPositionSubsystem) {
    addCommands(
        new ParallelDeadlineGroup(new ArmRollerOuttakeCommand(armRollersSubsystem),
            new ElevatorRequestSelectorCommand(elevatorSubsystem, elevatorPose),
            new SequentialCommandGroup(
                new WaitCommand(0.4).until(() -> elevatorSubsystem.isElevatorAtReq(elevatorPose)),
                new ArmRequestSelectorCommand(armPositionSubsystem,
                    ArmConstants.kArmIntakeAndScorePos))),
        new ParallelDeadlineGroup(new SequentialCommandGroup(
            new WaitCommand(0.2)
                .until(() -> armPositionSubsystem.armAtReq(ArmConstants.kArmRestPos)),
            new ElevatorRequestSelectorCommand(elevatorSubsystem, ElevatorConstants.kOriginPosition)
                .until(() -> elevatorSubsystem.isElevatorAtReq(ElevatorConstants.kOriginPosition))),
            new ArmRequestSelectorCommand(armPositionSubsystem, ArmConstants.kArmRestPos)));
  }
}
