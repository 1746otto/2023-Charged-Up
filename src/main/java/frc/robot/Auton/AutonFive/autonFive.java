package frc.robot.Auton.AutonFive;

import frc.robot.Constants;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.IntakeRollCommand;
import frc.robot.commands.LowGoalCommand;
import frc.robot.subsystems.Indexersubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
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
public class autonFive extends SequentialCommandGroup{
    public autonFive(Swerve s_Swerve){
        addCommands(new SequentialCommandGroup(new autonFiveTrajectory1(s_Swerve),
        new LowGoalCommand(new Indexersubsystem()), 
        new autonFiveTrajectory2(s_Swerve), 
        new IntakeExtendCommand(new IntakeExtendSubsystem()), 
        new IntakeRollCommand(new IntakeRollerSubsystem(), 
        new IntakeExtendSubsystem()), 
        new IntakeRetractCommand(new IntakeExtendSubsystem()), 
        new autonFiveTrajectory3(s_Swerve), 
        new LowGoalCommand(new Indexersubsystem())));
    }
    
}
