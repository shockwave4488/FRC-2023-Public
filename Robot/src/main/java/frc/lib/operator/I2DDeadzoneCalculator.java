package frc.lib.operator;

import edu.wpi.first.math.Pair;

public interface I2DDeadzoneCalculator {
  /**
   * Deadzone calculation with a given deadzone
   *
   * @param val The value to be deadzoned.
   * @param deadzone The amount that val inputs must be greater than in order for a return > 0.
   * @return The deadzoned value.
   */
  public Pair<Double, Double> deadzone(double xVal, double yVal, double deadzone);

  /**
   * @param val The value to be deadzoned.
   * @return The deadzoned value.
   */
  public Pair<Double, Double> deadzone(double xVal, double yVal);

  public boolean isPastDeadzone(double xVal, double yVal);
}
