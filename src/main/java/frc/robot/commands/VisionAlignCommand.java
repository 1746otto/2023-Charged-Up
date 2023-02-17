// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */

public class VisionAlignCommand extends CommandBase {
  private final VisionSubsystem m_visionSubsystem;
  private final Swerve m_Swerve;
  private double limeLightAngle = 12; // I think this is what we measued but not 100% sure, in degrees
  private double limeLightHeight = 14; // Place holder vale don't entirely remember
  private int targetType = 0; //0 is apriltags for scoring, 1 is tape for scoring, prob going to have to use 2 for tags on the double Substations
  private double scoringAprilTagHeight = 18.13; //in inches to middle of april tag 
  private double substationAprilTagHeight = 27.38; // in inches to middle of april tag
  private double scoringRRtapeHeightBottom = 24.125; //24 and 1/8 inches to middle of bottom rr tape, tape is approximately 4 inches high, placed at 1ft 10 1/8 inches
  private double scoringRRtapeHeightTop = 43.875; //in inches
  private double scoringHubEdge = 24.5; //guesstimate of how far limelight is from edge of hub when lined up, want actual measurements
  private double[] targetHeights = {scoringAprilTagHeight,scoringRRtapeHeightBottom,substationAprilTagHeight};
  private int m_tagID;
  private double m_xOffset; //in degrees. 50 for nothin. Kinda jank, could probably use something better but it is fine.
  private double targetAngle;
  private int driveDirection;
  private int rotDirection;
  private Pigeon2 m_gyro;
  private double initialAngle;
  //the direction constants are just guesses, going to need to actually test to be sure, I also don't know much about swerve so could be there as well
  private int ccw = -1;
  private int cw = 1;
  private int left = -1;
  private int right = 1;
  private double m_kRotp = Constants.Swerve.angleKP;
  private double m_kDrivep = Constants.Swerve.driveKP;
  private boolean shouldRotate = true;
  private boolean shouldAlign = false;
  private boolean shouldMoveTowards = false;
  private double currentAngle;
  Alliance m_allianceColor = DriverStation.getAlliance();

  private void setRotDirection(){
    if(currentAngle > targetAngle || (currentAngle - 180 <targetAngle)){
      rotDirection = ccw;
    }
    else{
      rotDirection = cw;
    }
  }
  private void setDriveDirection(){
    if (m_visionSubsystem.getXOffset() < 0){
      driveDirection = left;
    }
    else{
      driveDirection = right;
    }
  }
  private double calcDistanceInches(){
    return ((targetHeights[targetType] - limeLightHeight) / (Math.tan((limeLightAngle + m_visionSubsystem.getYOffset()) *(Math.PI/180))));
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAlignCommand(VisionSubsystem vision, Swerve Swerve) {
    m_Swerve = Swerve;
    m_visionSubsystem = vision;
    m_gyro = Swerve.gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_visionSubsystem, m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tagID = m_visionSubsystem.getTagID();
    //Carson's code below, commented out just in case needed for later and I misunderstood something
    // if (m_tagID != 0) {
    //     if (m_tagID <= 4) {
    //         targetAngle = 0;
    //     } 
    //     else {
    //         targetAngle = 180;
    //     }
    // }
    // if (m_gyro.getYaw() % 360 - targetAngle >= 0) {
    //     direction = -1;
    // }
    targetAngle = 0;
    // Will be used once we are able to use absolute angles on the field, probably set up wrong
    // if (m_allianceColor.equals(Alliance.Blue)){
    //   targetAngle = 180;
    // }
    // else if(m_allianceColor.equals(Alliance.Red)){
    //   targetAngle = 0;
    // }
    initialAngle = m_gyro.getYaw() % 360;
    targetType = (int) m_visionSubsystem.getGetPipe();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_gyro.getYaw() % 360;
    double rotationalVal = Math.abs(currentAngle - targetAngle);
    if (shouldRotate) {
      //I do not know how to use swerve commands so this portion is mainly an outline
      setRotDirection();
      //powerRotate(rotDirection * (rotationalVal * kRotp));
      if (Math.abs(rotationalVal) < 0.5){
        shouldRotate = false;
        shouldAlign = true;
      }
    }
    else if (!(Math.abs(m_visionSubsystem.getXOffset()) < 0.5) && shouldAlign){
      setDriveDirection();
      //powerstraf(driveDirection * (m_visionSusbsystem.getXOffset() *m_kDrivep));
      if (m_visionSubsystem.getXOffset() <0.5){
        shouldAlign = false;
        shouldMoveTowards = true;
        }
      }
      else if((Math.abs(calcDistanceInches()) > scoringHubEdge + 0.5) && shouldMoveTowards){
        //powerdrive(calcDistanceInches() * m_kDrivep);
        if (calcDistanceInches() < scoringHubEdge + 0.5){
          shouldMoveTowards = false;
        }
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
