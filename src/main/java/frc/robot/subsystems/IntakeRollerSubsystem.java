package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;;

public class IntakeRollerSubsystem extends SubsystemBase {
  CANSparkMax masterMotor;
  CANSparkMax slaveMotor;

  public IntakeRollerSubsystem() {
    masterMotor = new CANSparkMax(IntakeRollerConstants.kMasterMotor, MotorType.kBrushless);
    slaveMotor = new CANSparkMax(IntakeRollerConstants.kSlaveMotor, MotorType.kBrushless);
    slaveMotor.follow(masterMotor, true);
    masterMotor.setSmartCurrentLimit(40);
    slaveMotor.setSmartCurrentLimit(40);
  }

  public void setMotorSpeed(double speed) {
    masterMotor.set(speed);
  }

  public void setMotorIntakeSpeed() {
    setMotorSpeed(IntakeRollerConstants.kRollerIntakeSpeed);
  }

  public void setMotorOuttakeSpeed() {
    setMotorSpeed(IntakeRollerConstants.kRollerOuttakeSpeed);
  }

  public void setMotorStoppedSpeed() {
    setMotorSpeed(IntakeRollerConstants.kRollerStopSpeed);
  }
}
