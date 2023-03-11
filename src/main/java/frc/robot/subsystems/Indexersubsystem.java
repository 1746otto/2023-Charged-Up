package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;


public class Indexersubsystem extends SubsystemBase {

  CANSparkMax MotorTread;
  CANSparkMax Motor1;
  CANSparkMax Motor2;
  private final AnalogInput beambreak;

  public Indexersubsystem() {

    MotorTread = new CANSparkMax(IndexerConstants.kIndexerMotorT, MotorType.kBrushless);
    Motor1 = new CANSparkMax(IndexerConstants.kIndexerMotor, MotorType.kBrushless);
    Motor2 = new CANSparkMax(IndexerConstants.kIndexerMotor2, MotorType.kBrushless);
    Motor2.setInverted(true);
    beambreak = new AnalogInput(IndexerConstants.kbeambreak);
  }

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

  public void RunLowGoal() {
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

}


