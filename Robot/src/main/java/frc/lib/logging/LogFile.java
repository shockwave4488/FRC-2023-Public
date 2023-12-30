package frc.lib.logging;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class LogFile {

  private static final Charset charset = Charset.forName("UTF-8");
  public static final LogFile VOID = new LogFile();
  /**
   * Prints out an error if something attempts to write to this "file". Use this as a placeholder to
   * prevent NullPointerExceptions if a log file isn't initialized but should have been. NEVER
   * REDIRECT SYSTEM.ERR TO THIS TO PREVENT A STACK OVERFLOW
   */
  public static final LogFile VOID_WITH_ERROR = new LogFile();

  private static class Tracker {
    private final String trackerName;
    private final Supplier<Object> valueSupplier;
    private final int frequency;
    private long lastPrint;

    public Tracker(String trackerName, Supplier<Object> valueSupplier, int frequency) {
      this.trackerName = trackerName;
      this.valueSupplier = valueSupplier;
      this.frequency = frequency;
      this.lastPrint = 0;
    }

    public Optional<String> update(long time) {
      if (time - lastPrint > 1000.0 / frequency) {
        lastPrint = time;
        return Optional.of(trackerName + ": " + valueSupplier.get());
      }
      return Optional.empty();
    }
  }

  private final LogManager logger;
  private final String name;
  private final Path path;
  private final OutputStream out;
  private final List<Tracker> trackers;
  private int defaultFrequency;

  @SuppressFBWarnings("NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE")
  public LogFile(LogManager logger, String name, Path path) throws IOException {
    this.logger = logger;
    this.name = name;
    this.path = path;
    Files.createDirectories(path.toAbsolutePath().getParent());
    this.out =
        new BufferedOutputStream(Files.newOutputStream(this.path, StandardOpenOption.CREATE_NEW));
    this.trackers = new ArrayList<>();
    this.defaultFrequency = -1;
  }

  private LogFile() {
    this.logger = null;
    this.name = null;
    this.path = null;
    this.out = OutputStream.nullOutputStream();
    this.trackers = new ArrayList<>();
    this.defaultFrequency = -1;
  }

  public String getName() {
    return name;
  }

  public LogFile addTracker(String trackerName, Supplier<Object> valueSupplier, int frequency) {
    trackers.add(new Tracker(trackerName, valueSupplier, frequency));
    return this;
  }

  public LogFile addTracker(String trackerName, Supplier<Object> valueSupplier) {
    if (defaultFrequency == -1) {
      println(
          LogLevel.ERROR,
          "A tracker frequency was never supplied, defaulting to 1 for '" + name + "'");
      return addTracker(trackerName, valueSupplier, 1);
    }
    return addTracker(trackerName, valueSupplier, defaultFrequency);
  }

  public LogFile setDefaultFrequency(int frequency) {
    this.defaultFrequency = frequency;
    return this;
  }

  private void println(String line) {
    if (this == VOID_WITH_ERROR) {
      new Exception("ATTEMPTED TO LOG TO VOID_WITH_ERROR").printStackTrace();
      return;
    }
    try {
      out.write((line + "\n").getBytes(charset));
    } catch (IOException e) {
      logger.handleInternalError(e);
    }
  }

  void beginPhase(String phase) {
    println(LogLevel.HIGH.getPrefix() + "[" + getTimeString() + "] <----- " + phase + " ----->");
  }

  private void printlnToLogsOnly(LogLevel level, String line) {
    logger.onLogPrint(this, level, line);
    println(level.getPrefix() + "[" + getTimeString() + "] " + line);
  }

  public void println(LogLevel level, String line) {
    printlnToLogsOnly(level, line);
    if (level.isError()) {
      System.err.println(line);
      DriverStation.reportError(line, false);
      if (level == LogLevel.ERROR_CONSOLE) {
        Robot.getConsole().errorln(line);
      }
    }
  }

  public void println(LogLevel level, String context, Throwable e) {
    try {
      context = (context == null ? "" : context + ": ");
      String basicMsg = context + e.getClass().getName() + ": " + e.getMessage();

      try (StringWriter str = new StringWriter();
          PrintWriter writer = new PrintWriter(str)) {
        e.printStackTrace(writer);
        String fullMsg = context + str.toString();

        if (level.isError()) {
          System.err.println(fullMsg);
          DriverStation.reportError(fullMsg, false);
        }
        printlnToLogsOnly(level, fullMsg);
        if (level == LogLevel.ERROR_CONSOLE) {
          Robot.getConsole().errorln(basicMsg);
        }
      } catch (IOException e2) {
        // Impossible
        if (level.isError()) {
          e.printStackTrace();
          DriverStation.reportError(basicMsg, false);
        }
        printlnToLogsOnly(level, basicMsg);
        if (level == LogLevel.ERROR_CONSOLE) {
          Robot.getConsole().errorln(basicMsg);
        }
        e2.printStackTrace();
        DriverStation.reportError("Failed to fully log an error!", false);
        Robot.getConsole().errorln("Failed to fully log an error!");
      }
    } catch (RuntimeException e2) {
      // Ensure the error is printed somewhere, no matter what
      e.printStackTrace();
      throw e2;
    }
  }

  public void println(LogLevel level, Throwable e) {
    println(level, null, e);
  }

  public void update(long time) {
    List<String> trackerUpdates = new ArrayList<>();
    for (Tracker tracker : trackers) {
      tracker.update(time).ifPresent(trackerUpdates::add);
    }
    if (trackerUpdates.isEmpty()) {
      return;
    }
    println("[Tracker] [" + getTimeString(time) + "] " + String.join(" | ", trackerUpdates));
  }

  public void flush() {
    try {
      out.flush();
    } catch (IOException e) {
      logger.handleInternalError(e);
    }
  }

  private String getTimeString(long time) {
    long diff = time - logger.getStartMillis();
    String decimal = "" + diff % 1000;
    while (decimal.length() < 3) {
      decimal = "0" + decimal;
    }
    return diff / 1000 + "." + decimal;
  }

  private String getTimeString() {
    return getTimeString(System.currentTimeMillis());
  }
}
