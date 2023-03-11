package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class Indexersubsystem extends SubsystemBase {
    
    CANSparkMax MotorTread;
    CANSparkMax Motor1;
    CANSparkMax Motor2;
    private final AnalogInput beambreak;

    private boolean beambreakLastState = false;
    // private float beamBreakIntVolt;


    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final Color sheetColor = Color.kGreen;
    private final Color cubeColor = Color.kPurple;
    private final Color coneColor = Color.kYellow;


  
    
    public Indexersubsystem() {

        MotorTread = new CANSparkMax(IndexerConstants.kIndexerMotorT, MotorType.kBrushless);
        Motor1 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor2 = new CANSparkMax(IndexerConstants.kIndexerMotor2, MotorType.kBrushless);
        Motor2.setInverted(true);
        beambreak = new AnalogInput(IndexerConstants.kbeambreak);


    }
  
  


 /* 
    public int[] getColorRGB(){
      int red = m_colorSensor.getRed();
      int green = m_colorSensor.getGreen();
      int blue = m_colorSensor.getBlue();
      return new int[] {red,green,blue};
    public void DisengagePistons() {
      disengage.set(true);
    }

    public void engagePistons() {
      disengage.set(false);
    }
  */


    public void runMotor1Clockwise() {
        Motor1.set(IndexerConstants.speed);
      }
    public void runMotor2Counterclockwise() {
        Motor2.set(IndexerConstants.speed);
    }
    public void runMotorTread() {
        MotorTread.set(IndexerConstants.Tspeed);
    }

    public void runAllMotors() {
        runMotor1Clockwise();
        runMotor2Counterclockwise();
        runMotorTread();
      }
      public void runMotor1ClockwiseRev() {
        Motor1.set(IndexerConstants.reverseSpeed);
      }
    public void runMotor2CounterclockwiseRev() {
        Motor2.set(IndexerConstants.reverseSpeed);
    }
    public void runMotorTreadRev() {
        MotorTread.set(IndexerConstants.reverseSpeed);
    }
    
    public void runAllMotorsRev() {
        runMotor1ClockwiseRev();
        runMotor2CounterclockwiseRev();
        runMotorTreadRev();
      }
      public void runZeroPower() {

        MotorTread.stopMotor();
        Motor1.stopMotor();
        Motor2.stopMotor();
    }
    public void RunLowGoal(){
        runAllMotors();



    }
    public void turnOffIndexer() {
        runZeroPower();

    }
    public void turnOffShooter() {
        runZeroPower();

    }
  public boolean beambreakBroken() {
      return beambreak.getVoltage() > .67;
    }
  public void autoIndexer() {
      if (beambreakBroken()) {
        runZeroPower();
      } else {
        runAllMotors();
      }
    }

  
    @Override
    public void periodic(){
      //This code wouldn't have worked any ways
      //System.out.println(beambreak.getVoltage());
    }
}
  


    

