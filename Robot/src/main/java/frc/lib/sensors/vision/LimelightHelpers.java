// Adapted from LimelightLib, modified by Shockwave

package frc.lib.sensors.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.logging.LogFile;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightResults;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;
import java.util.function.BiFunction;
import java.util.function.Function;

public class LimelightHelpers {

  private static volatile ObjectMapper mapper;
  private static boolean robotIsReal = RobotBase.isReal();
  private static LogFile limelightLog = LogFile.VOID_WITH_ERROR;

  /** Print JSON Parse time to the console in milliseconds */
  static boolean profileJSON = false;

  public static void init(LogManager logger) {
    limelightLog = logger.getLogFile("LimelightHelpers");
  }

  static final String sanitizeName(String name) {
    if (name.isEmpty()) {
      return "limelight";
    }
    return name;
  }

  public enum CoordinateSpace {
    Field(inData -> inData /*, CoordinateSystem.NWU()*/),
    Robot(
        inData -> {
          inData[1] *= -1;
          return inData;
        } /*,
          new CoordinateSystem(CoordinateAxis.N(), CoordinateAxis.E(), CoordinateAxis.U())*/),
    Camera(
        inData ->
            new double[] {
              inData[2], -inData[0], -inData[1], inData[5], -inData[0], -inData[1]
            } /*,
              CoordinateSystem.EDN()*/),
    Target(
        inData ->
            new double[] {
              inData[2], inData[0], -inData[1], inData[5], -inData[0], -inData[1]
            } /*,
              new CoordinateSystem(CoordinateAxis.W(), CoordinateAxis.D(), CoordinateAxis.N())*/);

    private final Function<double[], double[]> dataTransformation;
    public final CoordinateSystem coordinateSystem;

    CoordinateSpace(
        Function<double[], double[]> dataTransformation /*, CoordinateSystem coordinateSystem*/) {
      this.dataTransformation = dataTransformation;
      // this.coordinateSystem = coordinateSystem;
      this.coordinateSystem = null;
    }

    public double[] convertLimelightData(double[] inData) {
      return dataTransformation.apply(inData);
    }
  }

  private static boolean checkTransformData(double[] inData, String poseType) {
    if (inData.length < 6) {
      limelightLog.println(LogLevel.ERROR, String.format("Bad LL {} Pose Data!", poseType));
      return false;
    }
    return true;
  }

  /**
   * @param inData Transform values from the Limelight NetworkTables or JSON dump
   * @param frame Coordinate frame that the {@code inData} transform values are relative to
   * @param objectMaker Geometry class constructor
   */
  static <T> T toGeometry3D(
      double[] inData,
      CoordinateSpace frame,
      BiFunction<Translation3d, Rotation3d, T> objectMaker) {
    if (!checkTransformData(inData, "3D")) {
      return objectMaker.apply(new Translation3d(), new Rotation3d());
    }
    inData = frame.convertLimelightData(inData);
    return objectMaker.apply(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Math.toRadians(inData[3]), Math.toRadians(inData[4]), Math.toRadians(inData[5])));
  }

  /**
   * @param inData Transform values from the Limelight NetworkTables or JSON dump
   * @param frame Coordinate frame that the {@code inData} transform values are relative to
   * @param objectMaker Geometry class constructor
   */
  static <T> T toGeometry2D(
      double[] inData,
      CoordinateSpace frame,
      BiFunction<Translation2d, Rotation2d, T> objectMaker) {
    if (!checkTransformData(inData, "2D")) {
      return objectMaker.apply(new Translation2d(), new Rotation2d());
    }
    inData = frame.convertLimelightData(inData);
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = Rotation2d.fromDegrees(inData[5]);
    return objectMaker.apply(tran2d, r2d);
  }

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  public static String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    URL url;
    try {
      url = new URL(urlString);
      return url;
    } catch (MalformedURLException e) {
      limelightLog.println(LogLevel.ERROR, "bad LL URL");
    }
    return null;
  }
  /////
  /////

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  public static double getTA(String limelightName) {
    return getLimelightNTDouble(limelightName, "ta");
  }

  public static double getLatency_Pipeline(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl");
  }

  public static double getLatency_Capture(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl_cap");
  }

  public static double getCurrentPipelineIndex(String limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  public static String getJSONDump(String limelightName) {
    return getLimelightNTString(limelightName, "json");
  }

  public static double[] getBotPose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotPose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  public static double[] getBotPose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  public static double[] getBotPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_targetSpace");
  }

  public static double[] getCameraPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
  }

  public static double[] getTargetPose_CameraSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
  }

  public static double[] getTargetPose_RobotSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
  }

  public static double[] getTargetColor(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  public static double getFiducialID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tid");
  }

  public static double getNeuralClassID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tclass");
  }

  public static boolean getTV(String limelightName) {
    return 1.0 == getLimelightNTDouble(limelightName, "tv");
  }

  /////
  /////

  public static void setPipelineIndex(String limelightName, int pipelineIndex) {
    setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
  }

  /** The LEDs will be controlled by Limelight pipeline settings, and not by robot code. */
  public static void setLEDMode_PipelineControl(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  public static void setLEDMode_ForceOff(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  public static void setLEDMode_ForceBlink(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  public static void setLEDMode_ForceOn(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  public static void setStreamMode_Standard(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  public static void setStreamMode_PiPMain(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  public static void setStreamMode_PiPSecondary(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 2);
  }

  /**
   * Sets the crop window. The crop window in the UI must be completely open for dynamic cropping to
   * work.
   */
  public static void setCropWindow(
      String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double[] entries = new double[4];
    entries[0] = cropXMin;
    entries[1] = cropXMax;
    entries[2] = cropYMin;
    entries[3] = cropYMax;
    setLimelightNTDoubleArray(limelightName, "crop", entries);
  }

  /////
  /////

  public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
    setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
  }

  public static double[] getPythonScriptData(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  /////
  /////

  /** Asynchronously take snapshot. */
  public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
    return CompletableFuture.supplyAsync(
        () -> {
          return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
  }

  private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
    URL url = getLimelightURLString(tableName, "capturesnapshot");
    try {
      HttpURLConnection connection = (HttpURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      if (!snapshotName.isEmpty()) {
        connection.setRequestProperty("snapname", snapshotName);
      }

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        return true;
      } else {
        limelightLog.println(LogLevel.ERROR, "Bad LL Request");
      }
    } catch (IOException e) {
      limelightLog.println(LogLevel.ERROR, e);
    }
    return false;
  }

  /** Parses Limelight's JSON results dump into a LimelightResults Object */
  public static LimelightResults getLatestResults(String limelightName) {

    long start = System.nanoTime();
    LimelightResults results = new LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    try {
      results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
    } catch (JsonProcessingException e) {
      // Prevent spam during simulation
      if (robotIsReal) {
        limelightLog.println(LogLevel.ERROR, e);
      }
    }

    long end = System.nanoTime();
    double millis = (end - start) * .000001;
    results.targetingResults.latency_jsonParse = millis;
    if (profileJSON) {
      System.out.printf("lljson: %.2f\r%n", millis);
    }

    return results;
  }
}
