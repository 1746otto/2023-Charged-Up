package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.FlapConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class FlapSubsystem extends SubsystemBase{
    private AnalogInput beamBreak;
    private boolean beamBreakLastState;
    private Solenoid piston;

    public FlapSubsystem(){
        piston = new Solenoid(PneumaticsModuleType.REVPH, FlapConstants.kFlapChannelID);
        beamBreak = new AnalogInput(ElevatorConstants.kElevatorAnalogInputChannel);
    }
    public boolean beamBreakBrokenForFlap(){
        return (beamBreakLastState);
    }
    public void flapOn(){
        piston.set(true);
    }
    public void flapOff(){
        piston.set(false);
    }

    @Override
    public void periodic(){
        beamBreakLastState = (Math.round(beamBreak.getVoltage()) == 0);
    }

}
