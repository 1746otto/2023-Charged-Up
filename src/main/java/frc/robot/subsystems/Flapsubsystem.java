package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlapConstants;
import frc.robot.constants.RobotConstants;



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

   