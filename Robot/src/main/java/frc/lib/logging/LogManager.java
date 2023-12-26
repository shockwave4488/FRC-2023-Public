package frc.lib.logging;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.misc.GCDetector;
import frc.lib.misc.MatchUtil;
import frc.robot.Robot;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public class LogManager {

  public static final Path allLogsPath =
      Path.of(
          Filesystem.getOperatingDirectory()
              + (RobotBase.isReal() ? "" : File.separator + "simulation")
              + File.separator
              + "logs");
  private static final int FLUSH_PERIOD = 5000;

  private final Path currentLogsPath;
  private final long startMillis;
  private final Map<String, LogFile> logs;
  private final LogFile mainLog;
  private MatchUtil.MatchPhase phase;
  private long lastFlush;

  public LogManager() {
    Date date = new Date();
    this.currentLogsPath =
        allLogsPath
            .resolve(new SimpleDateFormat("yyyy-MM-dd").format(date))
            .resolve(new SimpleDateFormat("HH-mm-ss-SSSS").format(date));
    this.startMillis = System.currentTimeMillis();
    this.logs = new HashMap<>();
    this.phase = null;
    this.mainLog = getLogFile("main");
    this.lastFlush = startMillis;

    Runtime.getRuntime().addShutdownHook(new Thread(this::flush));

    MatchUtil.runOnRealMatchDetermination(
        realMatch -> {
          mainLog.println(
              LogLevel.HIGH,
              "Match Type: "
                  + (realMatch ? "Real - " + DriverStation.getMatchType().name() : "Not Real"));
          if (realMatch) {
            try {
              Files.createFile(
                  currentLogsPath.resolve(DriverStation.getMatchType().name() + ".matchtype"));
            } catch (IOException e) {
              handleInternalError(e);
            }
          }
        });
  }

  public LogFile getLogFile(String name) {
    LogFile log = logs.get(name);
    if (log == null) {
      try {
        log = new LogFile(this, name, currentLogsPath.resolve(name + ".log"));
        log.beginPhase(phase == null ? "Disabled" : phase.toString());
      } catch (IOException e) {
        handleInternalError(e);
        log = LogFile.VOID;
      }
      logs.put(name, log);
    }
    return log;
  }

  public LogFile getMainLog() {
    return mainLog;
  }

  public LogManager withDefaultLoggers() {
    GCDetector gcDetector = new GCDetector();
    LogFile jvmLog = getLogFile("jvm").setDefaultFrequency(2);
    jvmLog.addTracker(
        "RAM",
        () ->
            (int)
                    ((1
                            - (double) Runtime.getRuntime().freeMemory()
                                / Runtime.getRuntime().totalMemory())
                        * 100)
                + "%");
    new CommandBase() {
      @Override
      public void execute() {
        if (gcDetector.checkHasGC()) {
          jvmLog.println(LogLevel.INFO, "GC Detected");
        }
      }
    }.withName("GCDetector").ignoringDisable(true).schedule();
    return this;
  }

  public long getStartMillis() {
    return startMillis;
  }

  public void beginPhase(MatchUtil.MatchPhase phase) {
    if (this.phase == phase) {
      return;
    }
    this.phase = phase;
    String phaseName = (phase == null ? "Disabled" : phase.toString());
    logs.values().forEach(log -> log.beginPhase(phaseName));
  }

  public void update() {
    long time = System.currentTimeMillis();
    logs.values().forEach(log -> log.update(time));
    if (time - lastFlush > FLUSH_PERIOD) {
      flush();
      lastFlush = time;
    }
  }

  public void flush() {
    logs.values().forEach(LogFile::flush);
  }

  void onLogPrint(LogFile file, LogLevel level, String line) {
    if (level.isImportant()) {
      if (file == mainLog) {
        return;
      }
      mainLog.println(level, "<" + file.getName() + "> " + line);
    }
    if (file == mainLog) {
      Robot.getConsole().debugln(line);
    }
  }

  void handleInternalError(IOException e) {
    e.printStackTrace();
    DriverStation.reportError(
        "Internal Logger Error: " + e.getClass().getName() + ": " + e.getMessage(),
        e.getStackTrace());
    Robot.getConsole().errorln("Internal Logger Error", e);
  }

  /**
   * @return "folder":{"folder":{...},"file":0,...}
   * @throws IOException
   */
  public String generateFileList() throws IOException {
    if (Files.exists(allLogsPath) && Files.isDirectory(allLogsPath)) {
      StringBuilder json = new StringBuilder();
      Files.walkFileTree(
          allLogsPath,
          new SimpleFileVisitor<Path>() {
            @SuppressFBWarnings(
                value = "NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE",
                justification = "allLogsPath and sub-paths can't be an empty path")
            @Override
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs)
                throws IOException {
              json.append('"');
              json.append(dir.getFileName().toString());
              json.append("\":{");
              return FileVisitResult.CONTINUE;
            }

            @Override
            public FileVisitResult postVisitDirectory(Path dir, IOException e) throws IOException {
              if (e != null) {
                throw e;
              }
              if (json.charAt(json.length() - 1) == ',') {
                json.deleteCharAt(json.length() - 1);
              }
              json.append("},");
              return FileVisitResult.CONTINUE;
            }

            @SuppressFBWarnings("NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE")
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
                throws IOException {
              json.append('"');
              json.append(file.getFileName().toString());
              json.append("\":0,");
              return FileVisitResult.CONTINUE;
            }
          });
      json.delete(0, "\"logs\":".length()); // Walk file tree includes the logs directory itself
      json.deleteCharAt(json.length() - 1); // The comma after the logs object
      return json.toString();
    }
    return "{}";
  }

  public void generateZip(OutputStream output) throws IOException {
    flush();
    try (ZipOutputStream zip = new ZipOutputStream(output)) {
      if (Files.exists(allLogsPath) && Files.isDirectory(allLogsPath)) {
        Files.walkFileTree(
            allLogsPath,
            new SimpleFileVisitor<Path>() {
              @Override
              public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs)
                  throws IOException {
                zip.putNextEntry(
                    new ZipEntry(allLogsPath.relativize(dir).toString().replace('\\', '/') + "/"));
                zip.closeEntry();
                return FileVisitResult.CONTINUE;
              }

              @Override
              public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
                  throws IOException {
                zip.putNextEntry(
                    new ZipEntry(allLogsPath.relativize(file).toString().replace('\\', '/')));
                Files.copy(file, zip);
                zip.closeEntry();
                return FileVisitResult.CONTINUE;
              }
            });
      }
      zip.flush();
    }
  }

  public boolean isLogFile(Path file) {
    if (!Files.exists(file) || !Files.isRegularFile(file)) {
      return false;
    }
    try {
      return file.toRealPath().startsWith(allLogsPath.toRealPath());
    } catch (IOException e) {
      handleInternalError(e);
      return false;
    }
  }

  /**
   * Checks if the file is contained in the folder this logger is outputting to Doesn't call {@link
   * #isLogFile(Path)}, which should be checked before interacting with files
   *
   * @param file The file to check
   * @return If the file is in use (and thus shouldn't be modified externally)
   */
  public boolean isLogFileInUse(Path file) {
    try {
      return file.toRealPath().startsWith(currentLogsPath.toRealPath());
    } catch (IOException e) {
      handleInternalError(e);
      return true;
    }
  }

  public Path getCurrentLogsPath() {
    return currentLogsPath;
  }
}
