package frc.lib.drive;

import frc.lib.preferences.PreferencesUtil;
import java.util.ArrayList;
import java.util.Map;
import org.json.simple.JSONObject;

public class SwerveParameters {
  public int driveMotorChannel;
  public int turningMotorChannel;
  public int turningEncoderChannel;
  public double absoluteEncoderOffset;
  public int absoluteEncoderResolution;
  public int relativeTurningEncoderResolution;
  public int driveEncoderResolution;
  public double wheelDiameter;
  public double driveGearRatio;
  public double turnSteerReduction;
  public ModulePosition modulePosition;
  public double moduleX;
  public double moduleY;

  public enum ModulePosition {
    FRONT_LEFT("FrontLeft"),
    FRONT_RIGHT("FrontRight"),
    BACK_LEFT("BackLeft"),
    BACK_RIGHT("BackRight");

    private final String niceName;

    private ModulePosition(String niceName) {
      this.niceName = niceName;
    }

    public String getNiceName() {
      return niceName;
    }
  }

  public static SwerveParameters[] getAllFromJSONObject(JSONObject swerveParametersJSON) {
    ArrayList<SwerveParameters> swerveParamList = new ArrayList<>();
    for (Object moduleParameters : swerveParametersJSON.entrySet()) {
      @SuppressWarnings("unchecked")
      SwerveParameters swerveParameters =
          getFromJSONObject((Map.Entry<String, Object>) moduleParameters);
      swerveParamList.add(swerveParameters);
    }
    return swerveParamList.toArray(SwerveParameters[]::new);
  }

  public static SwerveParameters getFromJSONObject(Map.Entry<String, Object> moduleParametersMap) {
    SwerveParameters parameters =
        PreferencesUtil.toObj((JSONObject) moduleParametersMap.getValue(), SwerveParameters.class);
    parameters.modulePosition = ModulePosition.valueOf(moduleParametersMap.getKey());
    return parameters;
  }

  /**
   * Information that will be passed to the SwerveModule
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningEncoderChannel ID for the turning encoder.
   * @param absoluteEncoderOffset Angle encoder offset value. Should be in ticks for neos and
   *     radians for falcons.
   * @param absoluteEncoderResolution Resolution of the absolute angle encoder used to determine the
   *     {@code absoluteEncoderOffset}.
   * @param relativeTurningEncoderResolution Resolution of the potentiometer/encoder used for
   *     turning PID control
   * @param driveEncoderResolution Resolution of the (likely integrated) encoder used for velocity
   *     control of the drive motor.
   * @param wheelDiameter Diameter of the wheel in meters.
   * @param driveGearRatio Gear ratio of the motor to wheel (how many times the motor spins for the
   *     wheel to spin once)
   * @param modulePosition Textual position of the module.
   * @param moduleX X coordinate of the module.
   * @param moduleY Y coordinate of the module.
   */
  public SwerveParameters(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double absoluteEncoderOffset,
      int absoluteEncoderResolution,
      int relativeTurningEncoderResolution,
      int driveEncoderResolution,
      double wheelDiameter,
      double driveGearRatio,
      double turnSteerReduction,
      ModulePosition modulePosition,
      double moduleX,
      double moduleY) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    this.turningEncoderChannel = turningEncoderChannel;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.absoluteEncoderResolution = absoluteEncoderResolution;
    this.relativeTurningEncoderResolution = relativeTurningEncoderResolution;
    this.driveEncoderResolution = driveEncoderResolution;
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.turnSteerReduction = turnSteerReduction;
    this.modulePosition = modulePosition;
    this.moduleX = moduleX;
    this.moduleY = moduleY;
  }

  public SwerveParameters() {
    // Called via Util.toObj()
  }
}
