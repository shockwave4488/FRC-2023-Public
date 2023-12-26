package frc.lib.logging;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

public enum LeveledSmartDashboard {
  HIGH,
  INFO;

  private static LeveledSmartDashboard manualMaxLevel = null;
  private static LeveledSmartDashboard prevDefaultMaxLevel = null;
  private static final List<BiConsumer<LeveledSmartDashboard, LeveledSmartDashboard>>
      changeListeners = new ArrayList<>();

  @SuppressFBWarnings(
      value = "EI_EXPOSE_STATIC_REP2",
      justification = "LeveledSmartDashboard isn't mutable")
  public static void setManualMaxLevel(LeveledSmartDashboard level) {
    LeveledSmartDashboard prev = getMaxLevel();
    manualMaxLevel = level;
    LeveledSmartDashboard now = getMaxLevel();
    if (prev != now) {
      callListeners(prev, now);
    }
  }

  @SuppressFBWarnings(
      value = "MS_EXPOSE_REP",
      justification = "Spotbugs thinks LeveledSmartDashboard is an array")
  public static LeveledSmartDashboard getManualMaxLevel() {
    return manualMaxLevel;
  }

  public static LeveledSmartDashboard getDefaultMaxLevel() {
    LeveledSmartDashboard output =
        switch (DriverStation.getMatchType()) {
          case None -> INFO;
          case Practice -> Robot.getConsole().isDebugEnabled() ? INFO : HIGH;
          case Qualification, Elimination -> HIGH;
        };
    if (prevDefaultMaxLevel == null) {
      prevDefaultMaxLevel = output;
    } else if (prevDefaultMaxLevel != output && manualMaxLevel == null) {
      LeveledSmartDashboard temp = prevDefaultMaxLevel;
      prevDefaultMaxLevel = output;
      callListeners(temp, output);
    }
    return output;
  }

  public static LeveledSmartDashboard getMaxLevel() {
    return manualMaxLevel == null ? getDefaultMaxLevel() : manualMaxLevel;
  }

  public static void addChangeListener(
      BiConsumer<LeveledSmartDashboard, LeveledSmartDashboard> listener) {
    changeListeners.add(listener);
  }

  private static void callListeners(LeveledSmartDashboard prev, LeveledSmartDashboard now) {
    changeListeners.forEach(listener -> listener.accept(prev, now));
  }

  public boolean isEnabled() {
    return ordinal() <= getMaxLevel().ordinal();
  }

  @Override
  public String toString() {
    return name().toLowerCase();
  }

  public void putData(Sendable sendable) {
    if (isEnabled()) {
      SmartDashboard.putData(sendable);
    }
  }

  public void putData(String name, Sendable sendable) {
    if (isEnabled()) {
      SmartDashboard.putData(name, sendable);
    }
  }

  public void putBoolean(String name, boolean value) {
    if (isEnabled()) {
      SmartDashboard.putBoolean(name, value);
    }
  }

  public void putNumber(String name, double value) {
    if (isEnabled()) {
      SmartDashboard.putNumber(name, value);
    }
  }

  public void putString(String name, String value) {
    if (isEnabled()) {
      SmartDashboard.putString(name, value);
    }
  }

  public void putBooleanArray(String name, boolean[] value) {
    if (isEnabled()) {
      SmartDashboard.putBooleanArray(name, value);
    }
  }

  public void putBooleanArray(String name, Boolean[] value) {
    if (isEnabled()) {
      SmartDashboard.putBooleanArray(name, value);
    }
  }

  public void putNumberArray(String name, double[] value) {
    if (isEnabled()) {
      SmartDashboard.putNumberArray(name, value);
    }
  }

  public void putNumberArray(String name, Double[] value) {
    if (isEnabled()) {
      SmartDashboard.putNumberArray(name, value);
    }
  }

  public void putStringArray(String name, String[] value) {
    if (isEnabled()) {
      SmartDashboard.putStringArray(name, value);
    }
  }
}
