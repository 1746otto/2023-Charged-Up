package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.ArmConstants;

public class ArmRollersSubsystem extends SubsystemBase {
  private CANSparkMax armRollerMotor;
  private double current;
  private double shootSpeed = -Math.abs(ArmConstants.kRollerShoot);

  public ArmRollersSubsystem() {
    armRollerMotor = new CANSparkMax(ArmConstants.kArmRollerMotorID, MotorType.kBrushless);
    armRollerMotor.setIdleMode(IdleMode.kBrake);
    // armRollerMotor.setSmartCurrentLimit(0);
  }

  public void armRollerIntake() {
    armRollerMotor.set(ArmConstants.kRollerSpeed);
  }

  public void armRollerStow() {
    armRollerMotor.set(ArmConstants.kRollerStowSpeed);
  }

  public void armRollerOuttake() {
    armRollerMotor.set(-Math.abs(ArmConstants.kRollerSpeed));
  }

  public void armRollerShoot() {
    // armRollerMotor.set(-Math.abs(ArmConstants.kRollerShoot));
    armRollerMotor.set(shootSpeed);
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
    SmartDashboard.putNumber("Shoot speed", shootSpeed);
    if (SmartDashboard.getNumber("Shoot speed", shootSpeed) != shootSpeed) {
      shootSpeed = -Math.abs(SmartDashboard.getNumber("Shoot speed", shootSpeed));
    }
  }
}
