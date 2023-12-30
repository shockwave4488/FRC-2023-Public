package frc.lib.logging;

import frc.lib.dashboard.actions.Action;

/**
 * Represents a window that the driver will be able to see messages in <br>
 * Debug methods will only show messages when debug mode is enabled <br>
 * Call debug methods when a message could be helpful later, but you don't want to spam the console
 */
public interface Console {
  public static final Console VOID =
      new Console() {
        @Override
        public void print(String msg) {}

        @Override
        public void error(String msg) {}

        @Override
        public void debug(String msg) {}

        @Override
        public boolean isDebugEnabled() {
          return false;
        }

        @Override
        public Console registerActionHandler(String name, Action handler) {
          return this;
        }

        @Override
        public Console registerActionHandler(String name, Runnable handler) {
          return this;
        }
      };

  public void print(String msg);

  public default void println(String msg) {
    print(msg + "\n");
  }

  public void error(String msg);

  public default void errorln(String msg) {
    error(msg + "\n");
  }

  public default void errorln(String msg, Throwable e) {
    errorln(msg + ": " + e.getClass().getName() + ": " + e.getMessage());
  }

  public void debug(String msg);

  public default void debugln(String msg) {
    debug(msg + "\n");
  }

  public boolean isDebugEnabled();

  public Console registerActionHandler(String name, Action handler);

  public Console registerActionHandler(String name, Runnable handler);
}
