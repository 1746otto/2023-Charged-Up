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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShootCubeHighAutonCommand extends SequentialCommandGroup {
  public ShootCubeHighAutonCommand(ElevatorSubsystem elevatorSub, ArmPositionSubsystem armPosSub,
      ArmRollersSubsystem armRollerSub) {
    // Total estimated time erring on the high side: .575 for elevator/arm + .125 for shoot + .525
    // for down = 1.225 + delays = ~1.375
    addCommands(
        new ParallelCommandGroup(
            new ElevatorRequestSelectorCommand(elevatorSub, ElevatorConstants.kHighCubePosition),
            new ArmRequestSelectorCommand(armPosSub, ArmConstants.ksubstationPosition)),
        // NO!!! Arbitrary Numbers, im gonna die
        // Arm Roller Command can be shortened
        // ~0.0625 min, but .125 will be safe. (I looked at the footage)
        new WaitCommand(0.5), new ArmRollerShootCommand(armRollerSub).withTimeout(.125),
        new ParallelCommandGroup(new ArmRequestSelectorCommand(armPosSub, ArmConstants.kArmRestPos),
            new ElevatorRequestSelectorCommand(elevatorSub, ElevatorConstants.kOriginPosition)),
        // This is to make sure elevator is settled a bit before we start running again.
        // May need to be upped to ~.15
        new WaitCommand(0.125));
  }
}
