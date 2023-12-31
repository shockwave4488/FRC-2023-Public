package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.drive.SwerveParameters;
import frc.lib.drive.SwerveParameters.ModulePosition;
import frc.lib.logging.LogManager;

/**
 * Base swerve module class. Subclasses must implement the methods {@link #getDriveRotations()},
 * {@link #getAngleTicks()}, {@link #getSpeedNative()}, and {@link
 * #setDesiredState(SwerveModuleState)}.
 */
public abstract class SwerveModule implements ISwerveModule {
  /** Diameter of the wheel in meters */
  protected final double wheelDiameter;
  /** Counts per revolution of the turning encoder */
  protected final int turningEncoderResolution;
  /** Counts per revolution of the drive encoder */
  protected final int driveEncoderResolution;
  /** Turning encoder/potentiometer angle offset */
  protected final double absoluteEncoderOffset;
  /** Drivetrain gear ratio */
  protected final double gearRatio;

  /** Desired module speed in m/s */
  protected double desiredModuleSpeed;
  /** Desired module angle in radians */
  protected double desiredModuleAngle;

  protected SwerveModuleState desiredState;

  /** Enum position of the module */
  protected final ModulePosition modulePosition;
  /** Coordinate of the module (where the origin is the robot center) */
  protected final Translation2d moduleLocation;

  /**
   * @param angleEncoderResolution Resolution of the encoder used for obtaining the actual angle of
   *     the module for PID control.
   */
  public SwerveModule(SwerveParameters parameters, LogManager logger, int angleEncoderResolution) {
    wheelDiameter = parameters.wheelDiameter;
    turningEncoderResolution = angleEncoderResolution;
    this.driveEncoderResolution = parameters.driveEncoderResolution;
    this.absoluteEncoderOffset = parameters.absoluteEncoderOffset;
    gearRatio = parameters.driveGearRatio;
    modulePosition = parameters.modulePosition;
    moduleLocation = new Translation2d(parameters.moduleX, parameters.moduleY);

    logger
        .getLogFile("SwerveModules/" + modulePosition.getNiceName())
        .setDefaultFrequency(3)
        .addTracker("Desired Speed", () -> desiredModuleSpeed)
        .addTracker("Desired Angle", () -> Math.toDegrees(desiredModuleAngle))
        .addTracker("Actual Speed", this::getSpeed)
        .addTracker("Actual Angle", this::getAbsoluteAngleDegrees);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), Rotation2d.fromRadians(getAbsoluteAngleRadians()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveRotations() * Math.PI * wheelDiameter / gearRatio,
        Rotation2d.fromRadians(getAbsoluteAngleRadians()));
  }

  /**
   * Sets which state the module should be in.
   *
   * @param desiredState State the module should be in. May change depending on where in the code it
   *     is used.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * Used internally in subclasses to set desired module variables.
   *
   * @return Current unbounded angle in radians
   */
  protected void setDesiredValues(SwerveModuleState desiredState, double currentAngleRadians) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngleRadians));

    desiredModuleSpeed = state.speedMetersPerSecond;
    desiredModuleAngle = state.angle.getRadians();
  }

  protected double metersPerSecToRPM(double metersPerSec) {
    return (metersPerSec * 60 * gearRatio) / (Math.PI * wheelDiameter);
  }

  @Override
  public double getDesiredAngle() {
    return desiredModuleAngle;
  }

  @Override
  public double getDesiredSpeed() {
    return desiredModuleSpeed;
  }

  /**
   * @return The RPM of the drive motor
   */
  protected abstract double getSpeedNative();

  @Override
  public double getSpeed() {
    return (getSpeedNative() / (60 * gearRatio)) * Math.PI * wheelDiameter;
  }

  /**
   * @return Turning encoder's position in native units.
   */
  protected abstract double getAngleTicks();

  /**
   * @return The angle of the module's wheel in radians (do not assume this is bounded)
   */
  public double getAngleRadians() {
    return getAngleTicks() * 2 * Math.PI / turningEncoderResolution;
  }

  /**
   * @return The angle of the module within the range (-pi, pi]
   */
  public double getAbsoluteAngleRadians() {
    return MathUtil.angleModulus(getAngleRadians());
  }

  @Override
  public double getAbsoluteAngleDegrees() {
    return Math.toDegrees(getAbsoluteAngleRadians());
  }

  /**
   * @return Drive encoder's position in revolutions (not accounted for gear ratio).
   */
  protected abstract double getDriveRotations();

  @Override
  public String getRobotPosition() {
    return modulePosition.name();
  }

  @Override
  public Translation2d getLocation() {
    return moduleLocation;
  }
}
