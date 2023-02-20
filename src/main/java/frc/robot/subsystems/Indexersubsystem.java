package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RobotConstants;



public class Indexersubsystem extends SubsystemBase {
    
    CANSparkMax MotorTread;
    CANSparkMax Motor1;
    CANSparkMax Motor2;
    private final Solenoid extend;
    private final Solenoid disengage;
    private final Solenoid pistons;
    private final AnalogInput beambreak;

    private boolean beambreakLastState = false;


  
    


    public Indexersubsystem() {

        MotorTread = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor1 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor2 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor2.setInverted(true);
        pistons =new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH, IndexerConstants.kChannel);
        extend = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        IndexerConstants.kExtendSolenoidChannel);
        disengage = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        IndexerConstants.kRetractSolenoidChannel);
        beambreak = new AnalogInput(IndexerConstants.kbeambreak);


    }

    public void DisengagePistons() {
      disengage.set(true);
    }

    public void engagePistons() {
      disengage.set(false);
    }



    public void runMotor1Clockwise() {
        Motor1.set(IndexerConstants.speed);
      }
    public void runMotor2Counterclockwise() {
        Motor2.set(IndexerConstants.speed);
    }
    public void runMotorTread() {
        MotorTread.set(IndexerConstants.speed);
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
        DisengagePistons();



    }


    public void turnOffIndexer() {
        runZeroPower();

    }
    public void turnOffShooter() {
        runZeroPower();
        engagePistons();

    }
    public boolean beambreakBroken() {
      return beambreakLastState;
    }
    public void autoIndexer() {
      if (beambreakBroken()) {
        runZeroPower();
      } else {
        runAllMotors();
      }
    }
    





}
  


    

