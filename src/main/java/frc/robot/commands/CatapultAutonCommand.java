package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.basic.CatapultRunToMax;
import frc.robot.commands.basic.CatapultRunToMin;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultAutonCommand extends SequentialCommandGroup {
  public CatapultAutonCommand(CatapultSubsystem catapultSubsystem) {
    addCommands(new CatapultRunToMax(catapultSubsystem).withTimeout(0.5),
        new CatapultRunToMin(catapultSubsystem));
  }
}
