package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.controlsystems.SimPID;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.Potentiometer;

/** A SwerveModule class that operates Spark Neos */
public class SwerveModuleNeos extends SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final Potentiometer m_turningEncoder;

  private final SparkMaxPIDController m_drivePIDController;
  private final SimPID m_turningPIDController;

  private static final double DEFAULT_DRIVE_P = 0.00005;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.00016;
  private static final double DEFAULT_TURN_P = 0.0006;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0.0001;

  /**
   * Constructs a SwerveModuleNeo.
   *
   * @param parameters Module-specific parameters
   */
  public SwerveModuleNeos(SwerveParameters parameters, LogManager logger, PreferencesParser prefs) {
    super(parameters, logger, parameters.absoluteEncoderResolution);
    m_driveMotor = new CANSparkMax(parameters.driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(parameters.turningMotorChannel, MotorType.kBrushless);
    m_turningEncoder = new Potentiometer(parameters.turningEncoderChannel);
    m_driveMotor.setClosedLoopRampRate(1);
    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(
        prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveP", DEFAULT_DRIVE_P));
    m_drivePIDController.setI(
        prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveI", DEFAULT_DRIVE_I));
    m_drivePIDController.setD(
        prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveD", DEFAULT_DRIVE_D));
    m_drivePIDController.setFF(
        prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveFF", DEFAULT_DRIVE_FF));
    m_turningPIDController =
        new SimPID(
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnP", DEFAULT_TURN_P),
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnI", DEFAULT_TURN_I),
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnD", DEFAULT_TURN_D));
    m_turningPIDController.setWrapAround(0, 4096);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    super.setDesiredState(desiredState);
    double currentAngleRadians = getAngleRadians();
    setDesiredValues(desiredState, currentAngleRadians);

    double targetAngleTicks = desiredModuleAngle / (2 * Math.PI) * turningEncoderResolution;

    m_turningPIDController.setDesiredValue(targetAngleTicks);
    double power =
        m_turningPIDController.calcPID(
            currentAngleRadians / (2 * Math.PI) * turningEncoderResolution);
    m_turningMotor.set(power);

    // Calculate the drive output from the drive PID controller.
    m_drivePIDController.setReference(
        metersPerSecToRPM(desiredModuleSpeed), CANSparkMax.ControlType.kVelocity);
  }

  /**
   * @return The analog input value of the encoder for the rotation motor
   */
  private double getAngleAnalog() {
    return m_turningEncoder.get();
  }

  /**
   * @return The value of the rotation encoder, adjusted for the potentiometer offset. Still returns
   *     values between 0 and the encoder resolution.
   */
  @Override
  protected double getAngleTicks() {
    return turningEncoderResolution
        - ((getAngleAnalog() - absoluteEncoderOffset + turningEncoderResolution)
            % turningEncoderResolution);
  }

  @Override
  protected double getDriveRotations() {
    return m_driveMotor.getEncoder().getPosition();
  }

  @Override
  public double getSpeedNative() {
    return m_driveMotor.getEncoder().getVelocity(); // get speed from spark
  }

  @Override
  public void updateSmartDashboard() {
    /*
    This is commented so we can quickly access this code if we need to debug our swerve modules,
    but don't have to devote the CPU cycles to always have this running.
    */
    // SmartDashboard.putNumber(modulePosition + " Desired Speed", getDesiredSpeed());
    // SmartDashboard.putNumber(modulePosition + " Desired Angle", getDesiredAngle());
    // SmartDashboard.putNumber(modulePosition + " Actual Speed", getSpeed());
    // SmartDashboard.putNumber(modulePosition + " Actual Angle", getAbsoluteAngleDegrees());
    // SmartDashboard.putNumber(modulePosition + " Angle Ticks", getAngleTicks());
    // SmartDashboard.putNumber(modulePosition + " Raw angle ticks", getAngleAnalog());
  }
}
