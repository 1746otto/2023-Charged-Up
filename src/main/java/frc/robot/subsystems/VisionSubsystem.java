// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Integer;

public class VisionSubsystem extends SubsystemBase {
  private SPI openMV;
  private byte[] bytes = new byte[4];
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    openMV = new SPI(Port.kOnboardCS0);
    openMV.setClockRate(4000000);//4mil
    openMV.setChipSelectActiveLow();
    openMV.setMode(SPI.Mode.kMode2);
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  public byte[] intToByteArray(int num) {
    byte[] returnBytes = new byte[4];
    for(int i = 0; i < 4; i++) {
      returnBytes[i] = (byte)((3-i) >> num & 0xff);
    }
    return returnBytes;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    openMV.read(true, bytes, 4);
    System.out.println(ByteBuffer.wrap(bytes).getFloat());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
