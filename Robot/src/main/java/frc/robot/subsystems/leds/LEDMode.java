package frc.robot.subsystems.leds;

public record LEDMode(int id, int arg) {
  public static LEDMode blank() {
    return new LEDMode(0);
  }

  /**
   * Set the entire strip to one color
   *
   * @param color The color in RGB format (0xFFAA00 is orange for example)
   */
  public static LEDMode solid(int color) {
    return new LEDMode(1, color);
  }

  public static LEDMode seismic() {
    return new LEDMode(2);
  }

  /**
   * @param speed The amount the animation progresses every iteration, where 256 is a complete loop
   *     (no movement)
   */
  public static LEDMode stripe(byte speed) {
    return new LEDMode(3, speed);
  }

  /**
   * @param speed The delay between every iteration
   */
  public static LEDMode rainbow(byte speed) {
    return new LEDMode(4, speed);
  }

  public LEDMode(int id) {
    this(id, 0);
  }
}
