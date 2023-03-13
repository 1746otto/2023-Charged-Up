package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClamperConstants;

public class ClamperSubsystem extends SubsystemBase {

  private Solenoid clamperPiston;

  public ClamperSubsystem() {
    clamperPiston = new Solenoid(ClamperConstants.kPH, PneumaticsModuleType.REVPH,
        ClamperConstants.kClamperChannel);
  }

  public void close() {
    clamperPiston.set(ClamperConstants.kClamperClosed);
  }

  public void open() {
    clamperPiston.set(ClamperConstants.kClamperOpen);
  }

  public boolean isClamperOpen() {
    return clamperPiston.get() == ClamperConstants.kClamperOpen;
  }
}
