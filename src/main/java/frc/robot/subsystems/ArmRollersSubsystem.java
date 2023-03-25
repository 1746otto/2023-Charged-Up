package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.ArmConstants;

public class ArmRollersSubsystem extends SubsystemBase {
  private CANSparkMax armRollerMotor;
  private double current;

  public ArmRollersSubsystem() {
    armRollerMotor = new CANSparkMax(ArmConstants.kArmRollerMotorID, MotorType.kBrushless);
    // armRollerMotor.setSmartCurrentLimit(0);
  }

  public void armRollerIntake() {
    armRollerMotor.set(ArmConstants.kRollerSpeed);
  }

  public void armRollerOuttake() {
    armRollerMotor.set(-ArmConstants.kRollerSpeed);
  }

  public void armRollerStop() {
    armRollerMotor.stopMotor();
  }

  public boolean currentBroken() {
    return (current >= ArmConstants.kArmCurrentMax);
  }

  @Override
  public void periodic() {
    current = armRollerMotor.getOutputCurrent();
  }
}
