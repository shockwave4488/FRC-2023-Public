package frc.lib.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

// Designed for the Sharp GP2Y0A21YK0F (short range) and Sharp GP2Y0A02YK (long range) sensors

public class IRBounceBack {

  private AnalogInput analog;

  public IRBounceBack(int port) {
    this.analog = new AnalogInput(0);
  }

  public double getBounceback() {
    // TODO add an interpolation table to this method similar to what we have on:
    // https://github.com/shockwave4488/FRC-2018-Public/blob/master/main/java/org/usfirst/frc/team4488/robot/sensors/AnalogBounceback.java

    double bouncebackValue = analog.getValue();
    return bouncebackValue;
  }
}
