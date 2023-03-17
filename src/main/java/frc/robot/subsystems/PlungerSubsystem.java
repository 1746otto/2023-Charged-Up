package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PlungerConstants;
import edu.wpi.first.wpilibj.Timer;

public class PlungerSubsystem extends SubsystemBase {

  private Solenoid plungerPiston;
  private Timer time;

  public PlungerSubsystem() {
    plungerPiston = new Solenoid(PlungerConstants.kPH, PneumaticsModuleType.REVPH,
        PlungerConstants.kPlungerChannel);
    time = new Timer();
  }

  public void extend() {
    plungerPiston.set(PlungerConstants.kPlungerExtended);
  }

  public void retract() {
    plungerPiston.set(PlungerConstants.kPlungerRetracted);
  }

  // Timer starts for 3 seconds to give time for plunger to finish before another command
  // starts
  public void Afterdelay() {
    time.delay(3);
  }

  public boolean isExtended() {
    return plungerPiston.get();
  }
}
