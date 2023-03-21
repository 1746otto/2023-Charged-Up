package frc.robot.Autos;

import frc.robot.commands.BalancingCommand;
import frc.robot.commands.DriveTo5DegreesCommand;
import frc.robot.commands.DriveBackTo5DegreesCommand;
import frc.robot.commands.DriveForwardsCommand;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;

public class BalanceAuton extends SequentialCommandGroup {
  public BalanceAuton(Swerve s_Swerve, BooleanSupplier robotCentricSup) {
    addCommands(new SequentialCommandGroup(new DriveTo5DegreesCommand(s_Swerve),
        new DriveForwardsCommand(s_Swerve), new DriveBackTo5DegreesCommand(s_Swerve),
        new BalancingCommand(s_Swerve)));
  }

}
