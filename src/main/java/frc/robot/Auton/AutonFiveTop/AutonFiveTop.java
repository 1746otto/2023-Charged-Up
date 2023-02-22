package frc.robot.Auton.AutonFiveTop;

import frc.robot.Constants;
import frc.robot.Auton.AutonFive.autonFiveBottomTrj2;
import frc.robot.Auton.AutonFive.autonFiveBottomTrj3;
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
public class AutonFiveTop extends SequentialCommandGroup{
    public AutonFiveTop(Swerve s_Swerve){
        addCommands(new SequentialCommandGroup(
        new LowGoalCommand(new Indexersubsystem()), 
        new AutonFiveTopTrj2(s_Swerve), 
        new IntakeExtendCommand(new IntakeExtendSubsystem()), 
        new IntakeRollCommand(new IntakeRollerSubsystem(), 
        new IntakeExtendSubsystem()), 
        new IntakeRetractCommand(new IntakeExtendSubsystem()), 
        new AutonFiveTopTrj3(s_Swerve), 
        new LowGoalCommand(new Indexersubsystem())));
    }
    
}

