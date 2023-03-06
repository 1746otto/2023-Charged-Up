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
    private final Solenoid piston;

    public PlungerSubsystem() {
        piston = new Solenoid(PlungerConstants.kModID, PneumaticsModuleType.REVPH, PlungerConstants.kPlungerChannel);
    }
 
    public boolean isEngaged() {
        return piston.get();
    }

    public void togglePlunger() {
        piston.set(true);
    }
}