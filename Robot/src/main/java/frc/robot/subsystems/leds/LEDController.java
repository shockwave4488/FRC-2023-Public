package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.lib.logging.LogManager;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.commands.LEDs.SetLEDMode;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class LEDController extends ShockwaveSubsystemBase {
  // Refer to top of LEDs.ino for protocol spec
  private class LEDDriver extends Thread {
    public LEDDriver() {
      setName("LEDDriver");
    }

    @Override
    public void run() {
      int index = queuedModes.size() - 1;
      for (int i = 0; i < index; i++) {
        queuedModes.remove();
      }
      LEDMode mode = queuedModes.remove();

      boolean[] data = new boolean[64];
      splitInt(mode.id(), data, 0);
      splitInt(mode.arg(), data, 32);

      long startTime = System.currentTimeMillis();
      boolean toggle = true;
      for (int i = 0; i < data.length; i++) {
        dataOutput.set(data[i]);
        indicatorOutput.set(toggle);
        while (indicatorInput.get() != toggle) {
          if (System.currentTimeMillis() - startTime > 1000) {
            indicatorOutput.set(false);
            return;
          }
        }
        toggle = !toggle;
      }
      indicatorOutput.set(false);
    }

    private void splitInt(int num, boolean[] target, int offset) {
      for (int i = 31; i >= 0; i--) {
        target[i + offset] = ((num & 0b1) != 0);
        num >>>= 1;
      }
    }
  }

  private final DigitalOutput indicatorOutput;
  private final DigitalOutput dataOutput;
  private final DigitalInput indicatorInput;
  private final Queue<LEDMode> queuedModes;
  private LEDMode mode;
  private LEDDriver driver;

  public LEDController(LEDMode defaultMode, int[] ledDioIds) {
    this.indicatorOutput = new DigitalOutput(ledDioIds[0]);
    this.dataOutput = new DigitalOutput(ledDioIds[1]);
    this.indicatorInput = new DigitalInput(ledDioIds[2]);
    this.queuedModes = new ConcurrentLinkedQueue<>();
    setDefaultCommand(new SetLEDMode(this, defaultMode));
  }

  /**
   * Sets the mode
   *
   * @param mode The new LED mode
   */
  public void setMode(LEDMode mode) {
    if (mode == null) {
      mode = LEDMode.blank();
    }
    if (this.mode != null && this.mode.id() == mode.id() && this.mode.arg() == mode.arg()) {
      return;
    }
    this.mode = mode;
    queuedModes.add(mode);
    if (driver == null || !driver.isAlive()) {
      driver = new LEDDriver();
      driver.start();
    }
  }

  /**
   * @return The current mode
   */
  public LEDMode getMode() {
    return mode;
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables(LogManager logger) {}

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}
}
