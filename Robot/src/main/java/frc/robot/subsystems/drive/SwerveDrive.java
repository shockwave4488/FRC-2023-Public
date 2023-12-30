package frc.robot.subsystems.drive;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.SwerveParameters.ModulePosition;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.sensors.gyro.NavX;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc.robot.commands.supercell.drive.BalanceOnChargeStation;
import frc.robot.constants.Constants.DriveTrainConstants;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Stream;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends ShockwaveSubsystemBase {
  private final Translation2d[] moduleLocations;
  private final ISwerveModule[] swerveModules;

  private final NavX m_gyro;
  private final Rotation2d gyroAdjustment;
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDrivePoseEstimator m_odometry;

  public final SubsystemBase driveRequirement = childRequirement();
  public final SubsystemBase rotationRequirement = childRequirement();
  public final SubsystemBase modifierRequirement = childRequirement();

  private boolean fieldRelative = true;
  private SwerveModifier modifier = SwerveModifier.forNone();
  private Optional<Double> overrideXSpeed = Optional.empty();
  private Optional<Double> overrideYSpeed = Optional.empty();
  private Optional<Double> overrideRotSpeed = Optional.empty();

  private SwerveModuleState[] curModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private Pose2d currentPose;
  private Field2d field = new Field2d();
  private double fieldPoseX;
  private double fieldPoseY;

  /**
   * The drive class for swerve robots
   *
   * @param gyro A NavX that's used to get the angle of the robot
   * @param modules Array of swerve modules in the order: front left, front right, back left, back
   *     right
   */
  public SwerveDrive(NavX gyro, Rotation2d gyroAdjustment, ISwerveModule[] modules) {
    m_gyro = gyro;
    this.gyroAdjustment = gyroAdjustment;
    swerveModules = modules;
    moduleLocations =
        Stream.of(modules).map(module -> module.getLocation()).toArray(Translation2d[]::new);
    m_kinematics = new SwerveDriveKinematics(moduleLocations);
    m_gyro.reset();
    m_odometry =
        new SwerveDrivePoseEstimator(
            m_kinematics, m_gyro.getYaw(), getModulePositions(), new Pose2d());
  }

  /**
   * @param xSpeed Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed Speed of the robot in the y direction (sideways) in m/s.
   * @param rot Angular rate of the robot in radians/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param centerOffset Offset from the robot center for point of rotation
   */
  public void assignModuleStates(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= modifier.moveScale();
    ySpeed *= modifier.moveScale();
    rot *= modifier.rotationScale();
    Translation2d centerOffset = modifier.centerOffset();
    if (fieldRelative) {
      centerOffset = centerOffset.rotateBy(m_gyro.getYaw().unaryMinus());
    }
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            centerOffset);
    assignModuleStates(swerveModuleStates);
    LeveledSmartDashboard.INFO.putNumber("xSpeed", xSpeed);
    LeveledSmartDashboard.INFO.putNumber("ySpeed", ySpeed);
    LeveledSmartDashboard.INFO.putNumber("rot", rot);
  }

  public void assignModuleStates(Map<ModulePosition, SwerveModuleState> desiredStates) {
    assignModuleStates(
        Stream.of(swerveModules)
            .map(module -> desiredStates.get(ModulePosition.valueOf(module.getRobotPosition())))
            .toArray(SwerveModuleState[]::new));
  }

  /**
   * Used to directly set the state of (and move) the swerve modules
   *
   * @param desiredStates A list of desired states for each of the swerve modules, following the
   *     order passed into {@link SwerveDriveKinematics}.
   */
  public void assignModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED);
    curModuleStates = desiredStates;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  /**
   * @param xSpeed Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed Speed of the robot in the y direction (sideways) in m/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void setTranslationSpeeds(double xSpeed, double ySpeed, boolean fieldRelative) {
    overrideXSpeed = Optional.of(xSpeed);
    overrideYSpeed = Optional.of(ySpeed);
    this.fieldRelative = fieldRelative;
  }

  /**
   * @param rotSpeed Angular rate of the robot in radians/sec.
   */
  public void setRotationSpeed(double rotSpeed) {
    overrideRotSpeed = Optional.of(rotSpeed);
  }

  public void setModifier(SwerveModifier modifier) {
    this.modifier = modifier;
  }

  public void clearModifier() {
    setModifier(SwerveModifier.forNone());
  }

  public SwerveModifier getModifier() {
    return modifier;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(curModuleStates);
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getRobotRelativeChassisSpeeds();
    Translation2d fieldRelativeSpeeds =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(m_gyro.getYaw());
    return new ChassisSpeeds(
        fieldRelativeSpeeds.getX(),
        fieldRelativeSpeeds.getY(),
        chassisSpeeds.omegaRadiansPerSecond);
  }

  public void stop() {
    overrideXSpeed = Optional.of(0.);
    overrideYSpeed = Optional.of(0.);
    overrideRotSpeed = Optional.of(0.);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getYaw(), getModulePositions());
  }

  @Override
  public void onStart() {
    for (ISwerveModule module : swerveModules) {
      module.onStart();
    }
  }

  @Override
  public void onStop() {
    for (ISwerveModule module : swerveModules) {
      module.onStop();
    }
  }

  @Override
  public void zeroSensors() {
    m_gyro.reset();
    resetOdometry(m_odometry.getEstimatedPosition());
  }

  @Override
  public void updateSmartDashboard() {
    for (ISwerveModule module : swerveModules) {
      module.updateSmartDashboard();
    }
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseX", fieldPoseX);
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseY", fieldPoseY);
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseYaw", currentPose.getRotation().getDegrees());
    LeveledSmartDashboard.HIGH.putData("Field", field);
    LeveledSmartDashboard.HIGH.putNumber("Gyro yaw", m_gyro.getYaw().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("Gyro pitch", m_gyro.getPitch().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("Gyro roll", m_gyro.getRoll().getDegrees());
    // Relative to zero yaw
    LeveledSmartDashboard.HIGH.putNumber(
        "Gyro tilt", BalanceOnChargeStation.getTilt(m_gyro, gyroAdjustment).getDegrees());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    int loggingFrequency = 5;

    logger
        .getLogFile("SwerveDrive/Position")
        .setDefaultFrequency(loggingFrequency)
        .addTracker("Robot_X_Coordinate", () -> fieldPoseX)
        .addTracker("Robot_Y_Coordinate", () -> fieldPoseY)
        .addTracker("Robot_Angle_(degrees)", () -> m_gyro.getYaw().getDegrees());

    logger
        .getLogFile("SwerveDrive/Charge station gyro tilt")
        .setDefaultFrequency(loggingFrequency)
        .addTracker(
            "Gyro tilt", () -> BalanceOnChargeStation.getTilt(m_gyro, gyroAdjustment).getDegrees());

    /**
     * Keeping this here for now in case we need to sort logging files by data type
     *
     * <p>logger.makeLogFile("Swerve Module Desired Speeds", loggingFrequency) .withTrackable("Front
     * Left Desired Speed", m_frontLeft::getDesiredSpeed) .withTrackable("Front Right Desired
     * Speed", m_frontRight::getDesiredSpeed) .withTrackable("Back Left Desired Speed",
     * m_backLeft::getDesiredSpeed) .withTrackable("Back Right Desired Speed",
     * m_backRight::getDesiredSpeed);
     *
     * <p>logger.makeLogFile("Swerve Module Desired Angles", loggingFrequency) .withTrackable("Front
     * Left Desired Angle", m_frontLeft::getDesiredAngle) .withTrackable("Front Right Desired
     * Angle", m_frontRight::getDesiredAngle) .withTrackable("Back Left Desired Angle",
     * m_backLeft::getDesiredAngle) .withTrackable("Back Right Desired Angle",
     * m_backRight::getDesiredAngle);
     *
     * <p>logger.makeLogFile("Swerve Module Actual Speeds", loggingFrequency) .withTrackable("Front
     * Left Actual Speed", m_frontLeft::getSpeed) .withTrackable("Front RightActual Speed",
     * m_frontRight::getSpeed) .withTrackable("Back Left Actual Speed", m_backLeft::getSpeed)
     * .withTrackable("Back Right Actual Speed", m_backRight::getSpeed);
     *
     * <p>logger.makeLogFile("Swerve Module Actual Angles", loggingFrequency) .withTrackable("Front
     * Left Actual Angle", m_frontLeft::getAbsoluteAngleDegrees) .withTrackable("Front Right Actual
     * Angle", m_frontRight::getAbsoluteAngleDegrees) .withTrackable("Back Left Actual Angle",
     * m_backLeft::getAbsoluteAngleDegrees) .withTrackable("Back Right Actual Angle",
     * m_backRight::getAbsoluteAngleDegrees);
     */
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getOdometry() {
    return m_odometry.getEstimatedPosition();
  }

  public Rotation2d getGyroYaw() {
    return m_gyro.getYaw();
  }

  public Rotation2d getGyroAdjustment() {
    return gyroAdjustment;
  }

  private SwerveModulePosition[] getModulePositions() {
    return Stream.of(swerveModules)
        .map(module -> module.getPosition())
        .toArray(SwerveModulePosition[]::new);
  }

  public void consumeVisionEstimate(Pose2d visionMeasurement) {
    // TODO: Use actual timestamp
    m_odometry.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    updateOdometry();
    currentPose = m_odometry.getEstimatedPosition();
    field.setRobotPose(currentPose);
    fieldPoseX = currentPose.getX();
    fieldPoseY = currentPose.getY();

    checkAndAdjustSpeeds();

    setModuleStates(curModuleStates);

    overrideXSpeed = Optional.empty();
    overrideYSpeed = Optional.empty();
    overrideRotSpeed = Optional.empty();
  }

  public void checkAndAdjustSpeeds() {
    /*
    Note: Instead of all this, we could just convert the states in setModuleStates(SwerveModuleState[])
    to ChassisSpeeds, but that would cause an extra conversion to take place only to go right back
    to SwerveModuleStates. Inverse kinematics may not be a lossless conversion, so that method isn't
    ideal and we should limit performing inverse kinematics as much as possible.
    */
    List<Optional<Double>> presentForceSpeeds =
        List.of(overrideXSpeed, overrideYSpeed, overrideRotSpeed).stream()
            .filter(optionalSpeed -> optionalSpeed.isPresent())
            .toList();
    // All three speeds have been set, so we can ignore previous states
    if (presentForceSpeeds.size() == 3) {
      assignModuleStates(
          overrideXSpeed.get(), overrideYSpeed.get(), overrideRotSpeed.get(), fieldRelative);
      // Some of the speeds are the same, so perform forward kinematics to modify the previously
      // used ChassisSpeeds.
      // Don't change states anything if no speeds have been set this periodic loop
    } else if (!presentForceSpeeds.isEmpty()) {
      ChassisSpeeds prevSpeeds =
          fieldRelative ? getFieldRelativeChassisSpeeds() : getRobotRelativeChassisSpeeds();
      assignModuleStates(
          overrideXSpeed.orElse(prevSpeeds.vxMetersPerSecond).doubleValue(),
          overrideYSpeed.orElse(prevSpeeds.vyMetersPerSecond).doubleValue(),
          overrideRotSpeed.orElse(prevSpeeds.omegaRadiansPerSecond).doubleValue(),
          fieldRelative);
    }
  }

  /**
   * Sets your position on the field
   *
   * @param pose The position on the field you want the robot to think it's at
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getYaw(), getModulePositions(), pose);
  }
}
