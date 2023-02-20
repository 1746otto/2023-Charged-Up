package frc.robot.Autos;

import frc.robot.Constants;
import frc.robot.commands.BalancingCommand;
import frc.robot.commands.DriveToNeg5DegreesCommand;
import frc.robot.commands.UnbalancingCommand;
import frc.robot.commands.DriveBackToNeg5DegreesCommand;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.function.BooleanSupplier;

public class BalanceAuton extends SequentialCommandGroup{
    public BalanceAuton(Swerve s_Swerve, BooleanSupplier robotCentricSup){
        addCommands(new SequentialCommandGroup(new DriveToNeg5DegreesCommand(s_Swerve, robotCentricSup), 
        new UnbalancingCommand(s_Swerve, robotCentricSup), new DriveBackToNeg5DegreesCommand(s_Swerve, robotCentricSup),
         new BalancingCommand(s_Swerve, robotCentricSup)));
    }
    
}