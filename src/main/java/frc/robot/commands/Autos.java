// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SwerveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;


public final class Autos {
  Swerve swerve;
  VisionSubsystem visionSubsystem;
  InstantCommand resetGyroCommand;
  ElevatorSubsystem elevatorSubsystem;

  private boolean hasZeroed = false;


  public Autos(Swerve swerve, VisionSubsystem visionSubsystem,
      ElevatorSubsystem elevatorSubsystem) {

    this.elevatorSubsystem = elevatorSubsystem;
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;

    resetGyroCommand = new InstantCommand(() -> {
      if (DriverStation.getAlliance() == Alliance.Red && !hasZeroed) {
        swerve.gyro.setYaw(180);
        swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
            new Pose2d());
        hasZeroed = true;
      } else if (!hasZeroed) {
        swerve.gyro.setYaw(0);
        swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
            new Pose2d());
        hasZeroed = true;
      }

    }, this.swerve);
  }

  // We are putting the zeroing before the gyro because we don't want to have the issue of not
  // zeroing before the match
  public Command balance() {
    return new SequentialCommandGroup(resetGyroCommand, new DriveTo5DegreesCommand(swerve),
        new BalancingCommand(swerve));
  }

  public Command moveBalance() {
    return new SequentialCommandGroup(resetGyroCommand, new DriveOverChargeStationCommand(swerve),
        new DriveBackTo5DegreesCommand(swerve), new BalancingCommand(swerve));
  }

  public Command scoreOne() {
    // The reason we need these wait commands because the commands end when the solenoid is set to
    // true, not when the solenoid is actually fully in that state.
    return new SequentialCommandGroup(resetGyroCommand,

        new ParallelDeadlineGroup(
            new SequentialCommandGroup(new WaitCommand(4.0),
                new ParallelCommandGroup(new WaitCommand(.375)),
                new ParallelCommandGroup(new WaitCommand(.25)), new WaitCommand(.375)),
            new ElevatorRunToRequestCommand(elevatorSubsystem, ElevatorConstants.kHighPosition)),
        new ElevatorRunToRequestCommand(elevatorSubsystem, ElevatorConstants.kOriginPosition)
            .withTimeout(.5));
  }

  public Command scoreOneBalance() {
    return new SequentialCommandGroup(scoreOne(), new DriveTo5DegreesCommand(swerve),
        new BalancingCommand(swerve));
  }

  public Command scoreOneMove() {
    return new SequentialCommandGroup(scoreOne(), new DriveForwardsCommand(swerve));
  }

  public Command correctAlliance() {
    return resetGyroCommand;
  }

  public Command move() {
    return new DriveForwardsCommand(swerve).beforeStarting(resetGyroCommand);
  }

  public Command PseudoBruh() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        // Change "4" to valid path planner program
        PathPlanner.loadPathGroup("PathPlannerOuterAutonFiveTriangleSquareTriangle", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0), controllerGroup.get(1));



    return autonCommmand;
  }

  public Command Bruh() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        // Change "4" to valid path planner program
        PathPlanner.loadPathGroup("PathPlannerOuterAutonFiveTriangleSquareTriangleBalance", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0), controllerGroup.get(1));



    return autonCommmand;
  }

  public Command exampleAuto() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Example Path",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    // Then we use the position we got from vision to get our actual initial pose and make a
    // trajectory to go to it.
    // PathPlannerTrajectory goToStart = PathPlanner.generatePath(
    // new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared),
    // new PathPoint(new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY()),
    // Rotation2d.fromDegrees(0), m_swerve.getPose().getRotation()),
    // new PathPoint(
    // new Translation2d(pathGroup.get(0).getInitialState().poseMeters.getX(),
    // pathGroup.get(0).getInitialState().poseMeters.getY()),
    // pathGroup.get(0).getInitialState().poseMeters.getRotation(),
    // pathGroup.get(0).getInitialState().holonomicRotation));

    // Next we must pass the trajectory into a command that follows it.
    // Currently this commmand is commented out because we don't have a limelight.
    // PPSwerveControllerCommand goToStartCommand =
    // new PPSwerveControllerCommand(
    // goToStart,
    // m_swerve::getPose,
    // SwerveConstants.swerveKinematics,
    // new PIDController(0, 0, 0),
    // new PIDController(0, 0, 0),
    // new PIDController(0, 0, 0),
    // m_swerve::setModuleStates,
    // true,
    // m_swerve
    // )
    // ;

    // We then make a list of controller commands that can be accessed through the .get(int i)
    // method.
    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();

    for (PathPlannerTrajectory traj : pathGroup) {
      controllerGroup.add(new PPSwerveControllerCommand(traj, swerve::getPose,
          SwerveConstants.swerveKinematics, new PIDController(0, 0, 0), new PIDController(0, 0, 0),
          new PIDController(0, 0, 0), swerve::setModuleStates, true, swerve));
    }

    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("do sumthin", new InstantCommand(() -> {
      // do sumthin here
    }));
    eventMap.put("do sumthin else", new InstantCommand(() -> {
      // do sumthin else here
    }));

    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0),
        new FollowPathWithEvents(controllerGroup.get(1), pathGroup.get(1).getMarkers(), eventMap),
        controllerGroup.get(2));
    // Add the requirments for the command
    autonCommmand.addRequirements(swerve);


    return autonCommmand;

  }


  public Command driveForwards() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("driveforwardxvalue", true,
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    // Then we use the position we got from vision to get our actual initial pose and make a
    // trajectory to go to it.
    // PathPlannerTrajectory goToStart = PathPlanner.generatePath(
    // new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared),
    // new PathPoint(new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY()),
    // Rotation2d.fromDegrees(0), m_swerve.getPose().getRotation()),
    // new PathPoint(
    // new Translation2d(pathGroup.get(0).getInitialState().poseMeters.getX(),
    // pathGroup.get(0).getInitialState().poseMeters.getY()),
    // pathGroup.get(0).getInitialState().poseMeters.getRotation(),
    // pathGroup.get(0).getInitialState().holonomicRotation));

    // Next we must pass the trajectory into a command that follows it.
    // Currently this commmand is commented out because we don't have a limelight.
    // PPSwerveControllerCommand goToStartCommand =
    // new PPSwerveControllerCommand(
    // goToStart,
    // m_swerve::getPose,
    // SwerveConstants.swerveKinematics,
    // new PIDController(0, 0, 0),
    // new PIDController(0, 0, 0),
    // new PIDController(0, 0, 0),
    // m_swerve::setModuleStates,
    // true,
    // m_swerve
    // )
    // ;

    // We then make a list of controller commands that can be accessed through the .get(int i)
    // method.
    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(3.8, 0.2, 0), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0));



    return autonCommmand;


  }

  public Command PathPlannerInnerAuton5SquareTriangle() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("PathPlannerInnerAuton5SquareTriangle", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0));



    return autonCommmand;


  }

  public Command pathplannerOuterAuton2ConeCubeBalance() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("pathplannerOuterAuton2ConeCubeBalance", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0));



    return autonCommmand;


  }

  public Command PathPlannerOuterAutonConeBalance() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("PathPlannerOuterAutonConeBalance", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0));



    return autonCommmand;


  }

  public Command PathPlannerOuterAutonCubeBalance() {
    // This is the combined trajectories of autons we want to use.
    // Each trajectory we want to use is seperated by a stop point.
    // We store each path in the deploy/Path Planner/ folder.
    // You can have multiple constraints for each path, but for our purposes it is not required.

    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("PathPlannerOuterAutonCubeBalance", true,
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    swerve.gyro.setYaw(0);
    if (DriverStation.getAlliance() == Alliance.Red) {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          new Pose2d(
              pathGroup.get(0).getInitialHolonomicPose().getTranslation()
                  .plus(new Translation2d(3.8544499898, 0)),
              pathGroup.get(0).getInitialHolonomicPose().getRotation()
                  .plus(Rotation2d.fromDegrees(180))));
    } else {
      swerve.poseEstimator.resetPosition(swerve.gyro.getRotation2d(), swerve.getModulePositions(),
          pathGroup.get(0).getInitialHolonomicPose());
    }

    SmartDashboard.putString("Initial Pose", pathGroup.get(0).getInitialPose().toString());

    List<PPSwerveControllerCommand> controllerGroup = new ArrayList<>();
    int i = 0;
    for (PathPlannerTrajectory traj : pathGroup) {
      System.out.println(i);
      i++;
      controllerGroup.add(
          new PPSwerveControllerCommand(traj, swerve::getPose, SwerveConstants.swerveKinematics,
              new PIDController(2.2, 0.2, .05), new PIDController(2.2, 0.2, .05),
              new PIDController(2.95, 0, 0.1325), swerve::setModuleStates, true, swerve));
    }


    // Now we create an event map that will hold the name of the marker and the corresponding event.
    HashMap<String, Command> eventMap = new HashMap<>();


    // Make the auton command
    SequentialCommandGroup autonCommmand = new SequentialCommandGroup(
        // goToStartCommand,
        controllerGroup.get(0));



    return autonCommmand;


  }

}
