// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Integer;

public class VisionSubsystem extends SubsystemBase {
  //private SPI openMVSPI;
  private static final int bytesTransfered = 4;
  private SerialPort openMV;
  private byte[] bytes = new byte[bytesTransfered];
  private int[] buffer = new int[1];
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    // openMVSPI = new SPI(Port.kOnboardCS0);
    // openMVSPI.setClockRate(640000);//4mil
    // openMVSPI.setChipSelectActiveLow();
    // openMVSPI.setMode(SPI.Mode.kMode2);
    openMV = new SerialPort(9600, SerialPort.Port.kUSB1, 8);
    openMV.setFlowControl(SerialPort.FlowControl.kRtsCts);
    openMV.setTimeout(0.01);
    
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
  public String byteToHex(byte num) {
    char[] hexDigits = new char[2];
    hexDigits[0] = Character.forDigit((num >> 4) & 0xF, 16);
    hexDigits[1] = Character.forDigit((num & 0xF), 16);
    return new String(hexDigits);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //openMV.read(true, bytes, 20);
    // openMVSPI.read(true, bytes, 4);
    if (openMV.getBytesReceived() == bytesTransfered) {
      bytes = openMV.read(bytesTransfered);
    }
    System.out.println(ByteBuffer.wrap(bytes).getFloat());
    // System.out.print(byteToHex(bytes[0]));
    // System.out.print(byteToHex(bytes[1]));
    // System.out.print(byteToHex(bytes[2]));
    // System.out.println(byteToHex(bytes[3]));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
