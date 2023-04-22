package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRollersSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutonHighCubeCommand extends SequentialCommandGroup {
  public AutonHighCubeCommand(ElevatorSubsystem elevatorSubsystem,
      ArmRollersSubsystem armRollersSubsystem) {
    addCommands(new ElevatorRequestSelectorCommand(null, 0));
  }
}
