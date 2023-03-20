package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class IndexerTreadSubsystem extends SubsystemBase {

  CANSparkMax treadMotor;
  private final AnalogInput beamBreak;

  public IndexerTreadSubsystem() {
    treadMotor = new CANSparkMax(IndexerConstants.kIndexerTreadMotor, MotorType.kBrushless);
    beamBreak = new AnalogInput(IndexerConstants.kbeamBreak);
  }

  public void setMotorSpeed(double speed) {
    treadMotor.set(speed);
  }

  public void setMotorIntakeSpeed() {
    setMotorSpeed(IndexerConstants.kTreadScoreSpeed);
  }

  public void setMotorOuttakeSpeed() {
    setMotorSpeed(IndexerConstants.kTreadOuttakeSpeed * 2);
  }

  public void setMotorScoreSpeed() {
    setMotorSpeed(IndexerConstants.kTreadScoreSpeed);
  }

  public void setMotorStoppedSpeed() {
    setMotorSpeed(IndexerConstants.kTreadStopSpeed);
  }

  public boolean beambreakBroken() {
    return beamBreak.getVoltage() > IndexerConstants.kBeamBreakBrokenVoltage;
  }
}
