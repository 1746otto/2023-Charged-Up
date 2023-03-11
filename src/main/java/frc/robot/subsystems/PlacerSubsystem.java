package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase {

  private final Solenoid plungerPiston;
  private final Solenoid clamperPiston;

  public PlacerSubsystem() {
    plungerPiston = new Solenoid(PlacerConstants.kModID, PneumaticsModuleType.REVPH,
        PlacerConstants.kPlungerChannel);
    clamperPiston = new Solenoid(PlacerConstants.kModID, PneumaticsModuleType.REVPH,
        PlacerConstants.kClamperChannel);
  }

  public boolean isPlungerExtended() {
    return plungerPiston.get();
  }

  public void extendPlunger() {
    plungerPiston.set(true);
  }

  public void retractPlunger() {
    plungerPiston.set(false);
  }

  public boolean isClamperOpen() {
    return clamperPiston.get();
  }

  public void closeClamper() {
    clamperPiston.set(false);
  }

  public void openClamper() {
    clamperPiston.set(true);
  }
}
