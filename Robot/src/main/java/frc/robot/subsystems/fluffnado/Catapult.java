package frc.robot.subsystems.fluffnado;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class Catapult extends ShockwaveSubsystemBase {

  private final CANSparkMax mainMotor;
  private final CANSparkMax followMotor;
  private final LogManager logger;
  private SparkMaxPIDController pidController;
  private double GEAR_RATIO;
  private double kP;
  private double kI;
  private double kD;
  private double kIz;
  private double kFF;
  private double MAX_OUTPUT;
  private double MIN_OUTPUT;
  private double MAX_RPM;
  private int smartMotionSlot;
  public double MAX_CATAPULT_ANGLE;

  public Catapult(PreferencesParser prefs, int mainID, int followID, LogManager logger) {
    updateFromPrefs(prefs);
    this.logger = logger;
    mainMotor = new CANSparkMax(mainID, MotorType.kBrushless);
    followMotor = new CANSparkMax(followID, MotorType.kBrushless);
    followMotor.follow(mainMotor, true);
    mainMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mainMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) ((MAX_CATAPULT_ANGLE / 360) * GEAR_RATIO));
    pidController = mainMotor.getPIDController();
    mainMotor.getEncoder().setPosition(0.0);
    followMotor.getEncoder().setPosition(0.0);
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
  }

  public void setTargetAngle(double speed, double acceleration, Rotation2d goalAngle) {
    if (0 <= goalAngle.getDegrees()
        && goalAngle.getDegrees() <= MAX_CATAPULT_ANGLE
        && speed <= MAX_RPM
        && speed > 0) {
      pidController.setSmartMotionMaxVelocity(speed, smartMotionSlot);
      pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
      pidController.setSmartMotionMaxAccel(acceleration, smartMotionSlot);
      pidController.setReference(
          goalAngle.getRotations() * GEAR_RATIO, CANSparkMax.ControlType.kSmartMotion);
    } else {
      logger
          .getMainLog()
          .println(
              LogLevel.ERROR,
              "Invalid Catapult Mode, " + "angle:" + goalAngle.getDegrees() + "speed:" + speed);
    }
  }

  public void setTargetVelocity(double speed) {
    if (Math.abs(speed) <= MAX_RPM) {
      pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
      LeveledSmartDashboard.INFO.putNumber("Catapult desired speed", speed);
    } else {
      logger.getMainLog().println(LogLevel.ERROR, "Invalid Catapult Speed, " + "speed:" + speed);
    }
  }

  public double getVelocity() {
    return mainMotor.getEncoder().getVelocity();
  }

  public void periodic() {}

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(mainMotor.getEncoder().getPosition() / GEAR_RATIO);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber("Catapult angle", getAngle().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("Catapult speed", getVelocity());
  }

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("catapult")
        .setDefaultFrequency(5)
        .addTracker("angle", () -> getAngle().getDegrees())
        .addTracker("speed", () -> getVelocity());
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    GEAR_RATIO = prefs.getInt("CatapultGearRatio");
    kP = prefs.getDouble("CatapultP");
    kI = prefs.getDouble("CatapultI");
    kD = prefs.getDouble("CatapultD");
    kIz = prefs.getDouble("CatapultIZone");
    kFF = prefs.getDouble("CatapultFF");
    MAX_OUTPUT = prefs.getInt("CatapultMaxOutput");
    MIN_OUTPUT = prefs.getInt("CatapultMinOutput");
    MAX_RPM = prefs.getInt("CatapultMaxRPM");
    smartMotionSlot = prefs.getInt("CatapultSmartMotionSlot");
    MAX_CATAPULT_ANGLE = prefs.getDouble("CatapultMaxAngle");
  }
}
