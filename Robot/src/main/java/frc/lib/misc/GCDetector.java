package frc.lib.misc;

import java.lang.ref.WeakReference;

/** A sketchy way to detect garbage collection events. Not guaranteed to catch all or even any. */
public class GCDetector {

  private WeakReference<Object> ref;

  public GCDetector() {
    ref = new WeakReference<>(new Object());
  }

  public boolean checkHasGC() {
    Object value = ref.get();
    if (value != null) {
      return false;
    }
    ref = new WeakReference<>(new Object());
    return true;
  }
}
