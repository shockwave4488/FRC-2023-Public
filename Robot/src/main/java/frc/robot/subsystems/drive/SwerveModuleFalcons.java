package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.MagneticEncoder;

/**
 * A SwerveModule class that operates Falcon 500s.
 *
 * <p>Some of the logic in this class was used from Team 2910 Jack In The Bot's
 * (https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/subsystems/DrivetrainSubsystem.java)
 * and Swerve Drive Specialties'
 * (https://github.com/SwerveDriveSpecialties/swerve-lib/tree/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib)
 * open source swerve code.
 */
public class SwerveModuleFalcons extends SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final MagneticEncoder m_turningMagneticEncoder;

  private static final double DEFAULT_DRIVE_P = 0.0004;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.055;
  private static final double DEFAULT_TURN_P = 0.2;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0;

  private static final double DRIVE_LIMIT_CURRENT_THRESHOLD = 100; // 50
  private static final double DRIVE_LIMITED_CURRENT = 90; // 40
  private static final double TURN_LIMIT_CURRENT_THRESHOLD = 50;
  private static final double TURN_LIMITED_CURRENT = 40;
  private static final double LIMIT_CURRENT_AFTER = 0.1;
  private double turnSteerReduction;

  /**
   * Constructs a SwerveModuleFalcon.
   *
   * @param parameters Module-specific parameters
   */
  public SwerveModuleFalcons(
      SwerveParameters parameters, LogManager logger, PreferencesParser prefs) {
    super(parameters, logger, parameters.relativeTurningEncoderResolution);
    turnSteerReduction = parameters.turnSteerReduction;
    m_driveMotor = new TalonFX(parameters.driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_turningMotor = new TalonFX(parameters.turningMotorChannel);
    m_turningMotor.configFactoryDefault();
    m_driveMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, DRIVE_LIMITED_CURRENT, DRIVE_LIMIT_CURRENT_THRESHOLD, LIMIT_CURRENT_AFTER));
    m_turningMotor.setInverted(true);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, TURN_LIMITED_CURRENT, TURN_LIMIT_CURRENT_THRESHOLD, LIMIT_CURRENT_AFTER));
    m_turningMagneticEncoder =
        new MagneticEncoder(
            parameters.turningEncoderChannel,
            parameters.absoluteEncoderResolution,
            absoluteEncoderOffset,
            false,
            4095);
    m_driveMotor.config_kP(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveP", DEFAULT_DRIVE_P));
    m_driveMotor.config_kI(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveI", DEFAULT_DRIVE_I));
    m_driveMotor.config_kD(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveD", DEFAULT_DRIVE_D));
    m_driveMotor.config_kF(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveFF", DEFAULT_DRIVE_FF));
    m_turningMotor.config_kP(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnP", DEFAULT_TURN_P));
    m_turningMotor.config_kI(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnI", DEFAULT_TURN_I));
    m_turningMotor.config_kD(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnD", DEFAULT_TURN_D));
    zeroTurnMotors();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Depending on how the motor controller works, set() might need to be called even if the
    // setpoint stays the same. This logic should be revisited to verify its validity.
    // if (desiredState.equals(this.desiredState)) return;
    super.setDesiredState(desiredState);
    double currentAngleRadians = getAngleRadians();
    setDesiredValues(desiredState, currentAngleRadians);

    // Calculate the drive output from the drive PID controller.
    double speedRPM = metersPerSecToRPM(desiredModuleSpeed);
    double speedTicksPer100ms = (speedRPM * turningEncoderResolution) / (10 * 60);

    double currentAngleRadiansMod = MathUtil.angleModulus(currentAngleRadians); // domain (-pi, pi]

    // Get target angle by adding the delta of the desired bounded angle and actual bounded angle to
    // the current unbounded angle, and converting to ticks.
    double targetAngleTicks =
        (currentAngleRadians + MathUtil.angleModulus(desiredModuleAngle - currentAngleRadiansMod))
            * turningEncoderResolution
            / (2 * Math.PI * turnSteerReduction);

    m_driveMotor.set(ControlMode.Velocity, speedTicksPer100ms);
    m_turningMotor.set(ControlMode.Position, targetAngleTicks);
  }

  @Override
  protected double getDriveRotations() {
    return m_driveMotor.getSelectedSensorPosition() / driveEncoderResolution;
  }

  /**
   * Note, this method returns position of the integrated TalonFX encoder rather than the turning
   * mag encoder, because the integrated encoder is used for the actual PID control.
   *
   * @return The digital position value of the encoder for the rotation motor.
   */
  @Override
  protected double getAngleTicks() {
    return (m_turningMotor.getSelectedSensorPosition() * turnSteerReduction);
  }

  @Override
  protected double getSpeedNative() {
    return m_driveMotor.getSelectedSensorVelocity() * (60 * 10) / turningEncoderResolution;
  }

  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Drive Desired Speed", getDesiredSpeed());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Drive Actual Speed", Math.abs(m_driveMotor.getSelectedSensorVelocity()));
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Desired Angle Degrees", getDesiredAngle() * 180 / Math.PI);
    LeveledSmartDashboard.INFO.putNumber(modulePosition + " Drive Actual Speed", getSpeed());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Actual Ticks Angle", m_turningMotor.getSelectedSensorPosition());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Actual Degrees Angle", getAbsoluteAngleDegrees());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Mag Angle", m_turningMagneticEncoder.getAngleOffset());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Mag Angle Raw", m_turningMagneticEncoder.getAngle());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Turn Motor Velocity",
        Math.abs(m_turningMotor.getSelectedSensorVelocity()));
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " power", m_turningMotor.getMotorOutputPercent());
  }

  @Override
  public void onStart() {
    zeroTurnMotors();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void onStop() {
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
  }

  private void zeroTurnMotors() {
    double turningMotorOffset =
        m_turningMagneticEncoder.getAngleOffset() // radians
            * turningEncoderResolution
            / (2 * Math.PI * turnSteerReduction);
    m_turningMotor.setSelectedSensorPosition(turningMotorOffset);
  }
}
