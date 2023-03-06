// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.ExampleCommand;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;

public final class Autos {
    Swerve m_swerve;
    ScoringAlignCommand m_scoringAlignCommand;

    public Autos(Swerve swerve, ScoringAlignCommand alignCommand) {
        m_swerve = swerve;
        m_scoringAlignCommand = alignCommand;
    }

    public Command exampleAuto() {
        //This is the combined trajectories of autons we want to use.
        //Each trajectory we want to use is seperated by a stop point.
        //We store each path in the deploy/Path Planner/ folder.
        //You can have multiple constraints for each path, but for our purposes it is not required.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Example Path", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        //Then we use the position we got from vision to get our actual initial pose and make a trajectory to go to it.
        PathPlannerTrajectory goToStart = PathPlanner.generatePath(
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
            new PathPoint(new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY()), Rotation2d.fromDegrees(0), m_swerve.getPose().getRotation()),
            new PathPoint(new Translation2d(pathGroup.get(0).getInitialState().poseMeters.getX(), pathGroup.get(0).getInitialState().poseMeters.getY()), 
            pathGroup.get(0).getInitialState().poseMeters.getRotation(), pathGroup.get(0).getInitialState().holonomicRotation)
        );


        //Next we must pass the trajectory into a command that follows it.
        //Currently this commmand is commented out because we don't have a limelight.
        // PPSwerveControllerCommand goToStartCommand = 
        //     new PPSwerveControllerCommand(
        //         goToStart, 
        //         m_swerve::getPose, 
        //         Constants.Swerve.swerveKinematics, 
        //         new PIDController(0, 0, 0), 
        //         new PIDController(0, 0, 0), 
        //         new PIDController(0, 0, 0), 
        //         m_swerve::setModuleStates, 
        //         true, 
        //         m_swerve
        //     )
        // ;

        //We then make a list of controller commands that can be accessed through the .get(int i) method.
        List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
        
        for (PathPlannerTrajectory traj: pathGroup){
            controllerGroup.add(
                new PPSwerveControllerCommand(
                    traj, 
                    m_swerve::getPose, 
                    Constants.Swerve.swerveKinematics, 
                    new PIDController(0, 0, 0), 
                    new PIDController(0, 0, 0), 
                    new PIDController(0, 0, 0), 
                    m_swerve::setModuleStates, 
                    true, 
                    m_swerve
                )
            );
        }

        //Now we create an event map that will hold the name of the marker and the corresponding event.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("do sumthin", new InstantCommand(() -> {
            //do sumthin here
        }));
        eventMap.put("do sumthin else", new InstantCommand(() -> {
            //do sumthin else here
        }));

        //Make the auton command
        SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
            //goToStartCommand,
            controllerGroup.get(0),
            new FollowPathWithEvents(controllerGroup.get(1), pathGroup.get(1).getMarkers(), eventMap),
            controllerGroup.get(2)
        );
        //Add the requirments for the command
        autonCommmand.addRequirements(m_swerve);
        

        return autonCommmand;
        
    }
}
