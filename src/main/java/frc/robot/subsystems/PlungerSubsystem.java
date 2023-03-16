package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PlungerConstants;

public class PlungerSubsystem extends SubsystemBase {

  private Solenoid plungerPiston;

  public PlungerSubsystem() {
    plungerPiston = new Solenoid(PlungerConstants.kPH, PneumaticsModuleType.REVPH,
        PlungerConstants.kPlungerChannel);
  }

  public void extend() {
    plungerPiston.set(PlungerConstants.kPlungerExtended);
  }

  public void retract() {
    plungerPiston.set(PlungerConstants.kPlungerRetracted);
  }

  public boolean isExtended() {
    return plungerPiston.get();
  }
}
