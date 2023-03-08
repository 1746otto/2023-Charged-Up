package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlapConstants;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;



public class Flapsubsystem extends SubsystemBase {
    

 
    private final Solenoid pistons; 
 


  
    


    public Flapsubsystem() {

         pistons =new Solenoid(RobotConstants.kREVPH, PneumaticsModuleType.REVPH, FlapConstants.kChannel);
    


    }
   public boolean isEngaged(){
    return pistons.get();
   }
   public void closeFlap(){
    pistons.set(false);
   }

   public void openFlap(){
     pistons.set(true);
   }


}

   