package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BackupAlignCommand extends CommandBase {
    VisionSubsystem m_visionSubsystem;
    Swerve m_swerve;
    private final double nodeDepth = Units.inchesToMeters(16 + 12); //includes width of the robot with bumpers.
    private double currentAngle;
    private double targetAngle = 0;
    private double limeLightAngle = 12.8;// placeholder until get actual
    private double limeLightHeight = 14; // Place holder vale don't entirely remember
    private int targetType = 0; //0 is apriltags for scoring, 1 is tape for scoring, prob going to have to use 2 for tags on the double Substations
    private double scoringAprilTagHeight = 18.13; //in inches to middle of april tag 
    private double substationAprilTagHeight = 27.38; // in inches to middle of april tag
    private double scoringRRtapeHeightBottom = 24.125; //24 and 1/8 inches to middle of bottom rr tape, tape is approximately 4 inches high, placed at 1ft 10 1/8 inches
    private double scoringRRtapeHeightTop = 43.875; //in inches
    private double[] targetHeights = {scoringAprilTagHeight,scoringRRtapeHeightBottom,substationAprilTagHeight};
    private int m_tagID;
    private double m_xOffset; //in degrees. 50 for nothin. Kinda jank, could probably use something better but it is fine.
    private int driveDirection;
    private int rotDirection;
    private Pigeon2 m_gyro;
    private double initialAngle;
    //the direction constants are just guesses, going to need to actually test to be sure, I also don't know much about swerve so could be there as well
    private int ccw = 1;
    private int cw = -1;
    private int up = 1;
    private int down = -1;
    private int allianceDirection = 1;
    private double rotationalVal;
    private boolean shouldRotate = true;
    private boolean shouldAlign = false;
    private boolean shouldMoveTowards = false;
    Alliance m_allianceColor = DriverStation.getAlliance();
    private int aprilTagDirection = -1;

  private void setRotDirection(){
    if(currentAngle > targetAngle || (currentAngle - 180 <targetAngle)){
      rotDirection = cw;
    }
    else{
      rotDirection = ccw;
    }
  }
  private void setDriveDirection(){
    if (m_visionSubsystem.getXOffset() < 0){
      driveDirection = down * allianceDirection;
    }
    else{
      driveDirection = up * allianceDirection;
    }
  }
  private double calcDistanceMeters(){
    return (Units.inchesToMeters((targetHeights[targetType] - limeLightHeight) / (Math.tan((limeLightAngle + m_visionSubsystem.getYOffset()) *(Math.PI/180)))));
  }

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public BackupAlignCommand(VisionSubsystem vision, Swerve swerve) {
        m_visionSubsystem = vision;
        m_swerve = swerve;
        
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_visionSubsystem, m_swerve);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if (m_allianceColor == Alliance.Red){
        allianceDirection = -1;
        targetAngle = 180;
        aprilTagDirection = 1;
      }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (shouldRotate) {
      currentAngle = (m_gyro.getYaw() + 180) % 360 - 180;
      setRotDirection();
      rotationalVal = Math.abs(targetAngle - currentAngle);
      if (rotationalVal < 2){
        m_swerve.drive(new Translation2d(0,0), (rotDirection * rotationalVal/180), true, true);
      }
      }
      else if (shouldAlign){
        setDriveDirection();
        if (Math.abs(m_visionSubsystem.getXOffset()) < 0.05)
          m_swerve.drive(new Translation2d(0, (driveDirection * Math.abs(m_visionSubsystem.getXOffset()))/29.8/*max angle difference of april tag*/), 0, true, true);
        else {
          shouldAlign = false;
        };
      }
      else{
        shouldRotate = false;
        shouldAlign = true;
      }


    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !(shouldAlign && shouldRotate);
    }
  }