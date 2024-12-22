package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LightConstants;

public class PicoLEDModule implements AutoCloseable {
  private final Thread readThread;
  private final AtomicBoolean threadRunning = new AtomicBoolean(true);
  private int port = SerialPortJNI.serialInitializePort((byte)1);
  private DigitalOutput interrupt = new DigitalOutput(LightConstants.dioPin);

  public void threadMain() {   
    SerialPortJNI.serialSetBaudRate(port, LightConstants.baudRate);
    SerialPortJNI.serialSetDataBits(port, (byte)8);
    SerialPortJNI.serialSetParity(port, (byte)0);
    SerialPortJNI.serialSetStopBits(port, (byte)10);
    SerialPortJNI.serialSetWriteBufferSize(port, LightConstants.bufferSize);
    SerialPortJNI.serialSetWriteMode(port, (byte)1);
  }

  public void interrupt() {
    interrupt.set(true);
    new WaitCommand(0.001);
    interrupt.set(false);
  }

  public void setLED(int brightness, int mode, int r, int g, int b) {
    int r2 = r /= 2;
    int g2 = g /= 2;
    int b2 = b /= 2;


    final byte[] command = {(byte)brightness, (byte)mode, (byte)r, (byte)r2, (byte)g, (byte)g2, (byte)b, (byte)b2};
    SerialPortJNI.serialWrite(port, command, 8);
    interrupt();
  }
  
  public PicoLEDModule() {
    readThread = new Thread(this::threadMain);
    readThread.setName("PicoColourController");
    readThread.start();
  }

  @Override
  public void close() throws Exception {
    threadRunning.set(false);
    readThread.join();
  }
}