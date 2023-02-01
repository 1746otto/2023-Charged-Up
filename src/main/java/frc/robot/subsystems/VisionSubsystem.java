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
  private static final int baudRate = 100000000/128;//400MHz on the cpu, half that on the ahb, half that on the apb = 100MHz. a prescaler of 128 is closest to our desired baudrate
  private SPI openMVSPI;
  private SerialPort openMV;
  private byte[] bytes = new byte[bytesTransfered];
  private int[] buffer = new int[1];
  private ByteBuffer byteBuffer = ByteBuffer.allocate(bytesTransfered);
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    openMVSPI = new SPI(Port.kOnboardCS0);
    openMVSPI.setClockRate(baudRate);
    openMVSPI.setChipSelectActiveLow();
    openMVSPI.setMode(SPI.Mode.kMode2);
    openMVSPI.initAuto(bytesTransfered);
    openMVSPI.setAutoTransmitData(new byte[]{0x31}, 0);
    openMVSPI.startAutoRate(1.0/(double)baudRate);
    openMVSPI.initAccumulator(1.0/(double)baudRate, 0, bytesTransfered, 0, 0, 0, bytesTransfered*8, false, false);
    // try {
    //   openMV = new SerialPort(9600, SerialPort.Port.kUSB);
    //   System.out.println("Connected on kUSB");
    // }
    // catch (Exception e) {
    //     System.out.println("Failed, trying kUSB1");
    //     try {
    //       openMV = new SerialPort(9600, SerialPort.Port.kUSB1);
    //       System.out.println("Connected on kUSB1");
    //     }
    //     catch (Exception e1) {
    //       System.out.println("Failed, trying kUSB2");
    //       try {
    //         openMV = new SerialPort(9600, SerialPort.Port.kUSB2);
    //         System.out.println("Connected on kUSB2");
    //       }
    //       catch (Exception e2) {
    //         System.out.println("Failed all connections");
    //       }
    //     }
    // }
  
    // openMV.setTimeout(0.01);
    
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
    openMVSPI.readAutoReceivedData(byteBuffer, bytesTransfered, 0.01);
    //System.out.println(ByteBuffer.wrap(bytes).getFloat());
    System.out.println(String.format("%8s", Integer.toBinaryString(byteBuffer.getInt())).replace(' ', '0'));
    System.out.println(openMVSPI.getAccumulatorCount());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
