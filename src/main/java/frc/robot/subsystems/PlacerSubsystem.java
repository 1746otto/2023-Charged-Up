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
import frc.robot.Constants.PlacerConstants;
import frc.robot.Constants.RobotConstants;

public class PlacerSubsystem extends SubsystemBase {
    private final Solenoid pistons;
    private final Solenoid extend;
    private final Solenoid disengage;
    public PlacerSubsystem() {
        pistons =
        new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH, PlacerConstants.kChannel);
    extend = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        PlacerConstants.kExtendSolenoidChannel);
    disengage = new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH,
        PlacerConstants.kRetractSolenoidChannel);
  }

    
public boolean getEngaged() {
    return pistons.get() == PlacerConstants.kPlacerEngaged;
}
public void extendPlacerUp() {
    extend.set(true);
}
public void DisengagePlacer() {
    disengage.set(false);
}

public void extendPlacerOut(){
    extend.set(true);
}
public void extendPlacerIn() {
    disengage.set(false);
}
}