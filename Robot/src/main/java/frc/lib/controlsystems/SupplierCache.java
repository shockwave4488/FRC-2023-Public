package frc.lib.controlsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

/** Utility for updating a backing field on demand from a Supplier */
public class SupplierCache<T> {
  private final Supplier<T> supplier;
  private final Consumer<T> onUpdate;
  private T value;

  public SupplierCache(Supplier<T> supplier, Consumer<T> onUpdate) {
    this.supplier = supplier;
    this.onUpdate = onUpdate;
  }

  public SupplierCache(Supplier<T> supplier, Runnable onUpdate) {
    this(supplier, newValue -> onUpdate.run());
  }

  public SupplierCache(Supplier<T> supplier) {
    this(supplier, newValue -> {});
  }

  public void update() {
    value = supplier.get();
    onUpdate.accept(value);
  }

  public T current() {
    return value;
  }
}
