package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.ArmRequestSelectorCommand;
import frc.robot.commands.basic.ArmRollerIntakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmPositionSubsystem;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ConeIntakeCommand extends SequentialCommandGroup {
  public ConeIntakeCommand(ElevatorSubsystem elevatorSub, ArmPositionSubsystem armPosSub,
      ArmRollersSubsystem armRollerSub) {
    addCommands(
        new ParallelCommandGroup(
            new ElevatorRequestSelectorCommand(elevatorSub,
                ElevatorConstants.kConeElevatorIntakePos),
            new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmConeIntakePos)),
        new WaitCommand(0.5), new ArmRollerIntakeCommand(armRollerSub).withTimeout(0.5),
        new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmRestPos));
  }
}
