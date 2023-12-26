package frc.lib.misc;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Property<T> implements Supplier<T>, Consumer<T> {
  protected T value;
  protected final BiConsumer<T, T> onSet;

  /**
   * @param defaultValue Starting value for the property
   * @param onSet Consumer of the old and new property values (respectiely) to run every time the
   *     property is set
   */
  public Property(T defaultValue, BiConsumer<T, T> onSet) {
    value = defaultValue;
    this.onSet = onSet;
  }

  /**
   * @param defaultValue Starting value for the property
   * @param onSet Runnable to execute every time the property is set
   */
  public Property(T defaultValue, Runnable onSet) {
    this(defaultValue, (oldValue, newValue) -> onSet.run());
  }

  public Property(T defaultValue) {
    this(defaultValue, () -> {});
  }

  public void set(T value) {
    T oldValue = this.value;
    this.value = value;
    onSet.accept(oldValue, value);
  }

  @Override
  public T get() {
    return value;
  }

  @Override
  public void accept(T value) {
    set(value);
  }
}
