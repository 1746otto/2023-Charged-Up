package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;


public class IndexerRollerSubsystem extends SubsystemBase {

  CANSparkMax leftRollerMotor;
  CANSparkMax rightRollerMotor;

  public IndexerRollerSubsystem() {
    leftRollerMotor =
        new CANSparkMax(IndexerConstants.kIndexerLeftRollerMotor, MotorType.kBrushless);
    rightRollerMotor =
        new CANSparkMax(IndexerConstants.kIndexerRightRollerMotor, MotorType.kBrushless);
    rightRollerMotor.follow(leftRollerMotor, true);

  }

  public void setMotorSpeed(double speed) {
    leftRollerMotor.set(speed);
  }

  public void setMotorIntakeSpeed() {
    setMotorSpeed(IndexerConstants.kRollerIntakeSpeed);
  }

  public void setMotorOuttakeSpeed() {
    setMotorSpeed(IndexerConstants.kRollerOuttakeSpeed);
  }

  public void setMotorStoppedSpeed() {
    setMotorSpeed(IndexerConstants.kRollerStopSpeed);
  }

}


