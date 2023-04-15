package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatapultSubsystem extends SubsystemBase {
  Servo catapultServo;


  public CatapultSubsystem() {
    catapultServo = new Servo(1);

  }

  public void setServoToMax() {
    catapultServo.setAngle(180);
  }

  public void setServoToMin() {
    catapultServo.setAngle(0);
  }

  public void getRaw() {
    catapultServo.getRaw();
  }

  public void setSpeedToMax() {
    catapultServo.setSpeed(1);
  }

}
