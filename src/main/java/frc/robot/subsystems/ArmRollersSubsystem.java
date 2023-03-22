package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ArmConstants;

public class ArmRollersSubsystem extends SubsystemBase {
  private TalonFX armRollerMotor;

  public ArmRollersSubsystem() {
    armRollerMotor = new TalonFX(ArmConstants.kArmRollerMotorID);
    armRollerMotor.configStatorCurrentLimit(null);
  }

  public void armRollerIntake() {
    armRollerMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kRollerOutputSpeed);
  }

  public void armRollerOuttake() {
    armRollerMotor.set(TalonFXControlMode.PercentOutput, -ArmConstants.kRollerOutputSpeed);
  }

  public void armRollerStop() {
    armRollerMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
