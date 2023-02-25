// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ScoringAlignCommand;
import frc.robot.commands.ExampleCommand;

import java.util.ArrayList;
import java.util.List;

public final class Autos {
    Swerve m_swerve;
    ScoringAlignCommand m_scoringAlignCommand;

    public Autos(Swerve swerve, ScoringAlignCommand alignCommand) {
        m_swerve = swerve;
        m_scoringAlignCommand = alignCommand;
    }

    public Command exampleAuto() {
        PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("Example Path", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<PathPlannerTrajectory>();
        
        pathGroup.add(
            PathPlanner.generatePath(
                new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
                new PathPoint(new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY()), Rotation2d.fromDegrees(0), m_swerve.getPose().getRotation()),
                new PathPoint(new Translation2d(exampleTrajectory.getInitialState().poseMeters.getX(), exampleTrajectory.getInitialState().poseMeters.getY()), 
                exampleTrajectory.getInitialState().poseMeters.getRotation(), exampleTrajectory.getInitialState().holonomicRotation)
            )
        );
        
        return new SequentialCommandGroup(
            
        );
    }
}
