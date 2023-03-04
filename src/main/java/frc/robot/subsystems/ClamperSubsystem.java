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
import frc.robot.Constants.ClamperConstants;
import frc.robot.Constants.RobotConstants;

public class ClamperSubsystem extends SubsystemBase {
    private final Solenoid piston;

    public ClamperSubsystem() {
        piston = new Solenoid(ClamperConstants.kModID, PneumaticsModuleType.REVPH, ClamperConstants.kClamperChannel);
    }

    
    public boolean isEngaged() {
        return piston.get();
    }

    public void toggleClamper() {
        piston.toggle();
    }
}
