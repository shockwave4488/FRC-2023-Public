package frc.lib.controlsystems;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.controlsystems.DoneCycleMachineConditions.MachineCondition;
import frc.lib.controlsystems.DoneCycleMachineConditions.SimpleCondition;
import frc.lib.misc.Property;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Done cycle logic ensures that a certain condition is met for a desired amount of time before the
 * state is trusted and acted upon. A {@link DoneCycleMachine} conveniently sets up and keeps track
 * of done cycles, leaving the user's responsiblity only to update the machine on a periodic basis,
 * in order to retrieve the status of the machine.
 *
 * <p>Using this class is simple:
 *
 * <p>1. Create a {@link #DoneCycleMachine} instance in a constructor of a subsystem, command, etc.
 *
 * <p>2. Pick the condition that will increment the done cycles. "Sub-classifications" of {@link
 * DoneCycleMachine} are emulated through extensions of {@link MachineCondition}.
 *
 * <p>2. Establish the loop by calling {@link #run()} in any periodic function.
 *
 * <p>3. Call the appropriate methods to configure the machine or check stability.
 *
 * <p>4. Optionally, use {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard#putData(String,
 * Sendable) SmartDashboard.putData()} to easily show the current status and settings on the
 * dashboard.
 *
 * @see {@link DoneCycleMachineConditions}
 */
public class DoneCycleMachine<T extends MachineCondition> implements Sendable {
  public class Status {
    private int doneCycles = 0;

    /**
     * @return Whether the goal condition is currently satisfied. This is in contrast to {@link
     *     #isReady()}, which returns true if this has been true for {@link #minDoneCycles} amount
     *     of time.
     */
    public boolean isSatifyingCondition() {
      return shouldIncrementCycles.getAsBoolean();
    }

    /** Returns whether the goal condition has been satisfied for the desired amount of time. */
    public boolean isReady() {
      return doneCycles >= minDoneCycles.get();
    }

    public int getDoneCycleCount() {
      return doneCycles;
    }
  }

  /** Enable/disable the {@link DoneCycleMachine} loop. Will reset done cycle count. */
  public final Property<Boolean> enabled =
      new Property<>(
          true,
          (oldVal, newVal) -> {
            if (!newVal) resetDoneCycles();
          });

  private final T shouldIncrementCycles;
  private final Status status = new Status();
  private String sendableName = "DoneCycleMachine";

  // ----- Settings -----
  /** Set the minimum number of cycles in range/satisfying the goal condition to return ready. */
  public final Property<Integer> minDoneCycles = new Property<>(1);
  /**
   * Turn on/off subtract-cycles mode. Setting {@link #subtractCycleNum} enables this by itself, so
   * this property is mainly useful for disabling it later.
   */
  public final Property<Boolean> shouldSubtractCycles = new Property<>(false);
  /**
   * Specify the number of cycles by which to reduce the done cycle count after each iteration out
   * of done range or not meeting the goal condition. Automatically enables subtract-cycles mode.
   */
  public final Property<Integer> subtractCycleNum =
      new Property<>(0, () -> shouldSubtractCycles.set(true));

  public static DoneCycleMachine<SimpleCondition> fromSupplier(BooleanSupplier goalCondition) {
    return new DoneCycleMachine<>(new SimpleCondition(goalCondition));
  }

  public static <T extends MachineCondition> DoneCycleMachine<T> withMinCycles(
      T goalCondition, int minDoneCycles) {
    return new DoneCycleMachine<>(goalCondition)
        .configure(machine -> machine.minDoneCycles.set(minDoneCycles));
  }

  public static DoneCycleMachine<SimpleCondition> supplierWithMinCycles(
      BooleanSupplier goalCondition, int minDoneCycles) {
    return withMinCycles(new SimpleCondition(goalCondition), minDoneCycles);
  }

  /** Constructs a new {@link #DoneCycleMachine}. See class comment for guidance on how to use. */
  public DoneCycleMachine(T goalCondition) {
    goalCondition.setSourceMachine(this);
    shouldIncrementCycles = goalCondition;
  }

  /**
   * Decorator method for configuring the machine. Use {@link #configureCondition(Consumer)} if you
   * want to access the {@link MachineCondition} object instead of the machine itself.
   */
  public DoneCycleMachine<T> configure(Consumer<? super DoneCycleMachine<T>> modifier) {
    modifier.accept(this);
    return this;
  }

  /**
   * Decorator method for configuring the machine condition. Use {@link #configure(Consumer)} if (in
   * rare cases) you want to access the actual {@link DoneCycleMachine} object.
   */
  public DoneCycleMachine<T> configureCondition(Consumer<? super T> modifier) {
    modifier.accept(shouldIncrementCycles);
    return this;
  }

  /**
   * @return An object used to retrieve the status of the machine.
   */
  public Status getStatus() {
    return status;
  }

  /**
   * @return The {@link MachineCondition} object passed into the constructor.
   */
  @SuppressFBWarnings("EI_EXPOSE_REP")
  public T getCondition() {
    return shouldIncrementCycles;
  }

  /**
   * Sets the name of this machine. Returns the base class {@link DoneCycleMachine}, so this should
   * be called after any other configuration decoration.
   */
  public DoneCycleMachine<T> withName(String name) {
    sendableName = name;
    return this;
  }

  public String getName() {
    return sendableName;
  }

  /**
   * {@link #DoneCycleMachine} loop calculating done cycles. Must call periodically for the machine
   * to do its job.
   */
  public void run() {
    if (enabled.get()) {
      shouldIncrementCycles.update();
      if (getStatus().isSatifyingCondition()) {
        getStatus().doneCycles++;
      } else {
        lowerDoneCycles();
      }
    }
  }

  /**
   * Reset done cycle count to ensure {@link #minDoneCycles} more iterations must pass before being
   * ready.
   */
  public void resetDoneCycles() {
    getStatus().doneCycles = 0;
  }

  protected void lowerDoneCycles() {
    getStatus().doneCycles =
        shouldSubtractCycles.get()
            ? Math.max(getStatus().doneCycles - subtractCycleNum.get(), 0)
            : 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", enabled::get, enabled::set);
    builder.addStringProperty(
        "Done cycles",
        () ->
            String.valueOf(Math.min(getStatus().doneCycles, minDoneCycles.get()))
                + " / "
                + String.valueOf(minDoneCycles.get()),
        null);
    shouldIncrementCycles.addSendableProperties(builder);
  }
}
