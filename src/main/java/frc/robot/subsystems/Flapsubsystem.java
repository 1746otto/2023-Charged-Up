package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlapConstants;
import frc.robot.constants.RobotConstants;

public class FlapSubsystem extends SubsystemBase {
  private final Solenoid pistons;

  public FlapSubsystem() {
    pistons =
        new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH, FlapConstants.kChannel);
  }

  public boolean isEngaged() {
    return pistons.get();
  }

  public void closeFlap() {
    pistons.set(FlapConstants.kFlapClosed);
  }

  public void openFlap() {
    pistons.set(FlapConstants.kFlapOpen);
  }

}

