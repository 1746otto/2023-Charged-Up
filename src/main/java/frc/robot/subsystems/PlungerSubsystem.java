package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PlungerConstants;
import frc.robot.Constants.RobotConstants;

public class PlungerSubsystem extends SubsystemBase {
    private final Solenoid pistons;
    private final Solenoid extend;
    private final Solenoid disengage;
    public PlungerSubsystem() {
        pistons =
        new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH, PlungerConstants.kChannel);
    extend = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        PlungerConstants.kExtendSolenoidChannel);
    disengage = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        PlungerConstants.kRetractSolenoidChannel);
  }

    
public boolean getEngaged() {
    return pistons.get() == PlungerConstants.kPlacerEngaged;
}
public void extendPlacerUp() {
    extend.set(true);
}
public void DisengagePlacer() {
    disengage.set(false);
}


public boolean isAtTop() {
    return false;
}
}