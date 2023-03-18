// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.constants.SwerveConstants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeExtensionSubsystem;


public final class AutosHighAutonOne {
  Swerve m_swerve;
  // ScoringAlignCommand m_scoringAlignCommand;

  public AutosHighAutonOne(Swerve swerve) {
    m_swerve = swerve;
    // m_scoringAlignCommand = alignCommand;
  }

  public Command exampleAuto() {
        //This is the combined trajectories of autons we want to use.
        //Each trajectory we want to use is seperated by a stop point.
        //We store each path in the deploy/Path Planner/ folder.
        //You can have multiple constraints for each path, but for our purposes it is not required.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("pathplannerHighAutonOne", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
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
        
        PPSwerveControllerCommand exampleTrajectoryCommandPart1 = 
            new PPSwerveControllerCommand(
                pathGroup.get(0), 
                m_swerve::getPose, 
                SwerveConstants.swerveKinematics, 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                m_swerve::setModuleStates, 
                true, 
                m_swerve
            )
        ;
        PPSwerveControllerCommand exampleTrajectoryCommandPart2 = 
            new PPSwerveControllerCommand(
                pathGroup.get(1), 
                m_swerve::getPose, 
                SwerveConstants.swerveKinematics, 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                m_swerve::setModuleStates, 
                true, 
                m_swerve
            )
        ;

        PPSwerveControllerCommand exampleTrajectoryCommandPart3 = 
            new PPSwerveControllerCommand(
                pathGroup.get(2), 
                m_swerve::getPose, 
                SwerveConstants.swerveKinematics, 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                m_swerve::setModuleStates, 
                true, 
                m_swerve
            )
        ;

        //Now we create an event map that will hold the name of the marker and the corresponding event.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("intake on", new AutomaticIntakeClamperCommand);
        
        
        //Make the auton command
        SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
            //goToStartCommand,
            exampleTrajectoryCommandPart1,
            new FollowPathWithEvents(exampleTrajectoryCommandPart2, pathGroup.get(1).getMarkers(), eventMap),
            exampleTrajectoryCommandPart3
        );
        // Add the requirments for the command
        autonCommmand.addRequirements(m_swerve);
        

        return autonCommmand;
        
    }
}
