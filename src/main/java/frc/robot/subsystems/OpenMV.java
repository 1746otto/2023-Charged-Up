// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

public class OpenMV extends SubsystemBase {
  AnalogInput openMV;
  int intOpenMVValue;
  
  /** Creates a new ExampleSubsystem. */
  public OpenMV() {
    openMV = new AnalogInput(2);
    openMV.setAverageBits(8);//48-52@0 143-146@100 536-540@400 1025-1034@1000 1515-1522@1500 2006-2013@2000 2499-2515@2500 2992-3006@3000 3487-3503@3500 3981-3995@4000 4072-4086@4095
    //global sample rate is 50kHz

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public double parseRotation(int rawVal) {
    return ((rawVal & 0b1111111111)/1000.0) + (rawVal >> 10) - Math.PI/2.0;
  }

  @Override
  public void periodic() {
    intOpenMVValue = ((int)(((openMV.getAverageVoltage()*819.2*1.515151515152)-45.1258591)/0.98497929));
    if (intOpenMVValue > 4096){
      intOpenMVValue = 4096;
    }
    else if(intOpenMVValue < 0){
      intOpenMVValue = 0;
    }
    //System.out.println(parseRotation(intOpenMVValue));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
