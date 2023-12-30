package frc.lib.controlsystems;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.math.EpsilonUtil;
import frc.lib.misc.Property;
import java.util.function.BiPredicate;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DoneCycleMachineConditions {
  private DoneCycleMachineConditions() {}

  public abstract static class MachineCondition implements BooleanSupplier {
    protected DoneCycleMachine<?> machine;

    protected void update() {}

    protected final void setSourceMachine(DoneCycleMachine<?> machine) {
      this.machine = machine;
    }

    protected void addSendableProperties(SendableBuilder builder) {}
  }

  public static class SimpleCondition extends MachineCondition {
    protected final BooleanSupplier goalCondition;

    public SimpleCondition(BooleanSupplier goalCondition) {
      this.goalCondition = goalCondition;
    }

    @Override
    public boolean getAsBoolean() {
      return goalCondition.getAsBoolean();
    }
  }

  public abstract static class MachineConditionBase extends MachineCondition {
    protected final BooleanSupplier goalCondition = getGoalCondition();

    protected abstract BooleanSupplier getGoalCondition();

    @Override
    public boolean getAsBoolean() {
      return goalCondition.getAsBoolean();
    }
  }

  /**
   * Handles calculating the stability of a device (motor, or otherwise) around a setpoint. Instead
   * of manually setting up and keeping track of cycles in range and configuration, if you pass this
   * object into a {@link DoneCycleMachine} for the goal condition, the values will automatically be
   * watched and will affect the reported ready status.
   *
   * <p>A {@link DoneCycleMachine} using this {@link MachineCondition} extension offers a few
   * additional functionalities, notably the ability to provide actual and desired values which
   * determine whether the condition is satisfied. Use {@link #changeGoalCondition(BiPredicate)} to
   * change the default comparison between the actual and desired value: {@link
   * Object#equals(Object) equals()}.
   *
   * <p>Note: Operation of the {@link DoneCycleMachine} is intended to be done on the Command level,
   * or wherever the setpoint is determined. For this to be possible, the {@link DoneCycleMachine}
   * object should be exposed in some way to the operating class. {@link DoneCycleCommand} makes
   * this process quite convenient.
   */
  public static class ValueCondition<T> extends MachineConditionBase {
    private BiPredicate<T, T> goalPredicate = getDefaultPredicate();
    protected T curActualValue;
    protected T curDesiredValue;
    private final Supplier<? extends T> actualValueSupplier;
    private Supplier<? extends T> desiredValueSupplier = () -> curDesiredValue;

    /**
     * Callback invoked when assigning setpoint. A method reference may generate a warning if a
     * wrapper class of a primitive type needs to be unboxed. When in doubt, resort to a lambda
     * expression.
     */
    public final Property<Consumer<? super T>> deviceControlCallback =
        new Property<>(desiredVal -> {});

    public ValueCondition(
        Supplier<? extends T> actualValueSupplier, Supplier<? extends T> desiredValueSupplier) {
      this(actualValueSupplier);
      this.desiredValueSupplier = desiredValueSupplier;
      curDesiredValue = desiredValueSupplier.get();
    }

    public ValueCondition(Supplier<? extends T> actualValueSupplier) {
      this.actualValueSupplier = actualValueSupplier;
      curActualValue = actualValueSupplier.get();
    }

    @Override
    public void update() {
      curActualValue = actualValueSupplier.get();
      setDesiredValue();
    }

    @Override
    protected BooleanSupplier getGoalCondition() {
      return () -> goalPredicate.test(curActualValue, curDesiredValue);
    }

    protected BiPredicate<T, T> getDefaultPredicate() {
      return (actualVal, desiredVal) ->
          (actualVal == null) ? desiredVal == null : actualVal.equals(desiredVal);
    }

    /**
     * Changes how the actual and desired values are compared to each other to decide when to
     * increment the done cycle count.
     *
     * @param goalPredicate Comparator of actual and desired value returning a boolean. Actual value
     *     should be first in the predicate; desired should be second.
     */
    public void changeGoalCondition(BiPredicate<T, T> goalPredicate) {
      this.goalPredicate = goalPredicate;
    }

    /**
     * Assign setpoint for the device.
     *
     * @param desiredValue Setpoint
     * @param reset Whether to lower the done cycle count, even if current value would meet the new
     *     conditions.
     */
    public void setDesiredValue(T desiredValue, boolean reset) {
      curDesiredValue = desiredValue;
      if (reset || !getAsBoolean()) {
        machine.lowerDoneCycles();
      }
    }

    /**
     * Assign setpoint for the device. Current cycles will not necessarily be reset.
     *
     * @param desiredValue Setpoint
     */
    public void setDesiredValue(T desiredValue) {
      setDesiredValue(desiredValue, false);
    }

    protected void setDesiredValue() {
      curDesiredValue = desiredValueSupplier.get();
      deviceControlCallback.get().accept(curDesiredValue);
    }

    public T getDesiredValue() {
      return curDesiredValue;
    }

    public T getActualValue() {
      return curActualValue;
    }

    protected void addDesiredValueProperty(SendableBuilder builder) {
      builder.addStringProperty(
          "Desired value",
          () -> {
            T desiredVal = getDesiredValue();
            return (desiredVal != null) ? desiredVal.toString() : "null";
          },
          null);
    }

    @Override
    public void addSendableProperties(SendableBuilder builder) {
      addDesiredValueProperty(builder);
      builder.addStringProperty(
          "Actual value",
          () -> {
            T actualVal = getActualValue();
            return (actualVal != null) ? actualVal.toString() : "null";
          },
          null);
    }

    @Override
    public boolean getAsBoolean() {
      return goalCondition.getAsBoolean();
    }
  }

  /**
   * Extension of {@link ValueCondition} for numbers which offers a tolerance for error via {@link
   * #setDoneRange(double)} as is required for most applications.
   */
  public static class NumberCondition extends ValueCondition<Double> {
    public class DoneRangeProperty extends Property<Double> {
      DoneRangeProperty() {
        super(1e-12);
      }

      /**
       * Set done range.
       *
       * @param resetCycles Whether to lower the done cycle count, even if current value would be
       *     within the new range
       */
      public void setTolerance(double errorEpsilon, boolean resetCycles) {
        super.set(errorEpsilon);
        if (resetCycles || !getAsBoolean()) {
          machine.lowerDoneCycles();
        }
      }

      /** Set done range. Resets done cycle count. */
      public void setTolerance(double errorEpsilon) {
        setTolerance(errorEpsilon, true);
      }

      @Override
      public void set(Double value) {
        setTolerance(value);
      }
    }

    /**
     * The tolerance for the actual value to be accepted even if it isn't precisely at the desired
     * value.
     */
    public final DoneRangeProperty doneRange = new DoneRangeProperty();

    public NumberCondition(Supplier<Double> actualValue, Supplier<Double> desiredValue) {
      super(actualValue, desiredValue);
    }

    public NumberCondition(Supplier<Double> actualValue) {
      super(actualValue);
      curDesiredValue = 0.0;
    }

    @Override
    protected BiPredicate<Double, Double> getDefaultPredicate() {
      return (actualVal, desiredVal) -> {
        requireNonNullParam(actualVal, "actualVal", "DoneCycleMachine condition");
        requireNonNullParam(desiredVal, "desiredVal", "DoneCycleMachine condition");
        return EpsilonUtil.epsilonEquals(actualVal, desiredVal, doneRange.get());
      };
    }

    @Override
    protected void addDesiredValueProperty(SendableBuilder builder) {
      builder.addDoubleProperty(
          "Desired value",
          () -> {
            Double desiredVal = getDesiredValue();
            return (desiredVal != null) ? desiredVal.doubleValue() : -1;
          },
          this::setDesiredValue);
    }

    @Override
    public void addSendableProperties(SendableBuilder builder) {
      super.addSendableProperties(builder);
      builder.addDoubleProperty("Done range", doneRange::get, doneRange::set);
    }
  }
}
