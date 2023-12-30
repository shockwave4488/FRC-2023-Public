package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.lang.ref.SoftReference;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.stream.Stream;

public class Average<T> {

  private final T[] values;
  private final Function<T, Object> setupAverage; // Object so the intermediary type doesn't matter
  private final Function<Object, T> finishAverage;
  private final BinaryOperator<Object> adder;
  private final BiFunction<Object, Integer, Object> divider;
  private int index;
  private boolean filled;
  private SoftReference<T> lastOutput;

  public static Average<Pose2d> createForPose(int numPoses) {
    return new Average<Pose2d>(
        new Pose2d[numPoses],
        pose ->
            new double[] { // Convert to array to avoid Rotation2d messing with radian sums
              pose.getX(), pose.getY(), pose.getRotation().getRadians() // TODO: Circular average
            },
        avg -> new Pose2d(avg[0], avg[1], new Rotation2d(avg[2])),
        (a, b) -> new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]},
        (sum, len) -> new double[] {sum[0] / len, sum[1] / len, sum[2] / len});
  }

  @SuppressWarnings("unchecked") // Intermediary type required by constructor arguments
  public <I> Average(
      T[] values,
      Function<T, I> setupAverage,
      Function<I, T> finishAverage,
      BinaryOperator<I> adder,
      BiFunction<I, Integer, I> divider) {
    this.values = values;
    this.setupAverage = setupAverage::apply;
    this.finishAverage = intermediary -> finishAverage.apply((I) intermediary);
    this.adder = (a, b) -> adder.apply((I) a, (I) b);
    this.divider = (intermediary, len) -> divider.apply((I) intermediary, len);
    this.index = -1;
    this.filled = false;

    if (values.length == 0) {
      throw new IllegalArgumentException("The array of values to be averaged cannot be empty!");
    }
  }

  public Average(T[] values, BinaryOperator<T> adder, BiFunction<T, Integer, T> divider) {
    this(values, value -> value, value -> value, adder, divider);
  }

  public void addValue(T value) {
    values[++index] = value;
    if (index == values.length - 1) {
      index = -1;
      filled = true;
    }
    lastOutput = null;
  }

  public void reset() {
    filled = false;
    lastOutput = null;
  }

  public boolean isReady() {
    return filled;
  }

  public Optional<T> getValue() {
    if (!isReady()) {
      return Optional.empty();
    }
    if (lastOutput != null) {
      T lastOutputValue = lastOutput.get();
      if (lastOutputValue != null) {
        return Optional.of(lastOutputValue);
      }
    }

    T output =
        Stream.of(values)
            .map(setupAverage)
            .reduce(adder)
            .map(intermediary -> divider.apply(intermediary, values.length))
            .map(finishAverage)
            .orElseThrow();
    lastOutput = new SoftReference<>(output);
    return Optional.of(output);
  }
}
