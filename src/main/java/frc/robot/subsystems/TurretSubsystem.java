package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Vision;

public class TurretSubsystem extends SubsystemBase {
  // Flywheel
  public TalonSRX controller;
  public VictorSPX follower1;
  public VictorSPX follower2;
  public VictorSPX follower3;

  private int peakVelocity = 53471;
  private double rotation = 1023;
  private double Kf = (1.0 * rotation / peakVelocity); // IDK what this does, but I don't have time
                                                       // to figure it out.
  private double Kp = 0.18;
  private double Kd = 1.25;
  private double Ki = 0.0;

  // Turret

  public TalonSRX turretMotor;

  public static double kTurretTicksPerRevolution = 28000;
  public static final double kTurretTicksPerDegree = kTurretTicksPerRevolution / 360;
  private double kP = 0.05;
  private double kD = 0 * kP;
  private double error = 0;
  double prevError = 0;
  double deltaError = 0;
  double xOffset;
  double targetRPM;



  public TurretSubsystem() {
    controller = new TalonSRX(40);
    follower1 = new VictorSPX(42);
    follower2 = new VictorSPX(43);
    follower3 = new VictorSPX(41);

    controller.setInverted(true);
    follower1.setInverted(true);
    follower2.setInverted(false);
    follower3.setInverted(true);

    controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    controller.setSensorPhase(true);
    follower1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    follower1.setSensorPhase(false);
    follower2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    follower2.setSensorPhase(false);
    follower3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    follower3.setSensorPhase(false);

    follower1.follow(controller);
    follower2.follow(controller);
    follower3.follow(controller);

    controller.config_kF(0, Kf);
    controller.config_kP(0, Kp);
    controller.config_kD(0, Kd);
    controller.config_kI(0, Ki);

    // Turret
    turretMotor = new TalonSRX(50);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    turretMotor.setSensorPhase(false);
    turretMotor.config_kP(0, 1);

    SmartDashboard.putBoolean("enable turret", false);
    SmartDashboard.putNumber("Target RPM", 0);
  }

  public void configTurret() {
    Vision.getLastResult();

    if (Vision.lastResult.hasTargets()) {
      double target = 0;
      xOffset = Vision.bestTarget.getYaw();

      if (target - xOffset != error) {
        error = target - xOffset;
        deltaError = error - prevError;
        double kPnorm = -(kP * error + kD * deltaError);
        if (kPnorm < -0.5) {
          kPnorm = -0.5;
        } else if (kPnorm > 0.5) {
          kPnorm = 0.5;
        }
        turretMotor.set(ControlMode.PercentOutput, kPnorm);
        prevError = error;

        return;
      }
      deltaError = 0;
      return;
    } else {
      System.out.println("Target Not Valid!");
    }
    turretMotor.set(ControlMode.PercentOutput, 0);
    error = 0;
    prevError = 0;
    deltaError = 0;
  }

  public double getRPM() {
    return (controller.getSelectedSensorVelocity() / 6.8266666666666);
  }


  public void setRPM(int rpm) {
    double velRequest = rpm * 6.8266666666666;
    controller.set(ControlMode.Velocity, velRequest);
    targetRPM = rpm;
    SmartDashboard.putNumber("Target RPM", targetRPM);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", getRPM());
    if (SmartDashboard.getNumber("Target RPM", targetRPM) != targetRPM) {
      setRPM((int) SmartDashboard.getNumber("Target RPM", targetRPM));
    }
    if (SmartDashboard.getBoolean("enable turret", false)) {
      configTurret();
    }
  }

}
