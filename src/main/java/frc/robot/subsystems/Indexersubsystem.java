package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;



public class Indexersubsystem extends SubsystemBase {
    CANSparkMax Motor;
    CANSparkMax Motor1;
    CANSparkMax Motor2;
    


    public Indexersubsystem() {

        Motor = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor1 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
        Motor2 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);

    }

    public void runCustomPower(double input) {

        Motor1.set(input);
        Motor2.set(input);
        Motor.set(input);


    }

    public void runZeroPower() {

        Motor.stopMotor();
        Motor1.stopMotor();
        Motor2.stopMotor();
    }
    public void turnOnIndexer() {
        runCustomPower(IndexerConstants.kIndexerMotor);

    }

    public void turnOffIntake() {
        runZeroPower();

    }
    





}
  


    

