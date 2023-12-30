package frc.robot.robotspecifics.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BaseRobotContainer;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.CommandUtil;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.lib.dashboard.DashboardServer;
import frc.lib.dashboard.IDashboardSupportedContainer;
import frc.lib.dashboard.gui.CameraWidget;
import frc.lib.dashboard.gui.FieldWidget;
import frc.lib.dashboard.gui.GroupWidget;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.math.JSONPosition;
import frc.lib.operator.CircularDeadzone;
import frc.lib.operator.I2DDeadzoneCalculator;
import frc.lib.operator.IDeadzoneCalculator;
import frc.lib.operator.POVRange;
import frc.lib.operator.SquareDeadzoneCalculator;
import frc.lib.preferences.PreferencesParser;
import frc.lib.preferences.PreferencesUtil;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.robot.Robot;
import frc.robot.autonomous.modes.eruption.AutonomousChooser;
import frc.robot.autonomous.modes.eruption.AutonomousTrajectories;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc.robot.commands.drive.RotateTowardsPosition;
import frc.robot.commands.drive.StandardDrive;
import frc.robot.commands.drive.StandardRotation;
import frc.robot.commands.drive.SwerveModifierCommand;
import frc.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc.robot.commands.drive.VisionPoseUpdater;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleNeos;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveRobotContainer extends BaseRobotContainer
    implements IDashboardSupportedContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final CommandXboxController driverJoystick;
  private final CommandGenericHID buttonBox;
  private final ISwerveModule[] swerveModules;
  private final Limelight limelight;
  private final AutonomousChooser autonomousChooser;
  private final AutoPIDControllerContainer pidControllers;

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        PreferencesUtil.toObj(limelightPositionJSON, JSONPosition.class).toTransform());
  }

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public SwerveRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);

    SwerveParameters[] swerveParameters =
        SwerveParameters.getAllFromJSONObject(prefs.getJSONObject("SwerveParameters"));
    swerveModules = new ISwerveModule[swerveParameters.length];
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i] = new SwerveModuleNeos(swerveParameters[i], logger, prefs);
    }

    swerve =
        new SwerveDrive(
            m_gyro, Rotation2d.fromDegrees(prefs.getDouble("GyroAdjustment")), swerveModules);
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);

    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    limelight =
        new Limelight(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);
    autonomousChooser =
        new AutonomousChooser(
            new AutonomousTrajectories(swerve, logger),
            swerve,
            null,
            null,
            null,
            null,
            m_gyro,
            limelight,
            logger,
            prefs);

    pidControllers =
        new AutoPIDControllerContainer(
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new ProfiledPIDController(
                prefs.getDouble("AutoTurnP"),
                prefs.getDouble("AutoTurnI"),
                prefs.getDouble("AutoTurnD"),
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI)));

    swerve.driveRequirement.setDefaultCommand(
        new StandardDrive(
            swerve,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            getDriverLeftStickInput(),
            driverJoystick.start()));
    swerve.rotationRequirement.setDefaultCommand(
        new StandardRotation(
            swerve,
            -DriveTrainConstants.SWERVE_ROTATION_SPEED,
            () -> squareDeadzone.deadzone(driverJoystick.getRightX())));
    swerve.modifierRequirement.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(swerve::clearModifier, swerve.modifierRequirement)
            .withName("ResetSwerveModifier"));
    addSubsystems();
    configureButtonBindings();

    new Trigger(RobotState::isEnabled)
        .onTrue(VisionPoseUpdater.createForLimelight(swerve, limelight));
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    DoneCycleCommand<RotateTowardsPosition> testCommand =
        new DoneCycleCommand<>(
                new RotateTowardsPosition(
                    swerve,
                    m_gyro,
                    pidControllers.thetaPidController,
                    () -> new Translation2d(1.5, 1.5)),
                false)
            .withDoneCycles(
                cmd ->
                    cmd.getComposedCommand().getYawDoneCycleMachine(12, Rotation2d.fromDegrees(5)));

    if (LeveledSmartDashboard.INFO.isEnabled()) {
      testCommand.putOnDashboard();
    }
    driverJoystick.b().toggleOnTrue(testCommand);

    driverJoystick
        .leftTrigger(0.75)
        .toggleOnTrue(
            new SwerveModifierCommand(swerve)
                .bindModifierToggle(buttonBox.button(1), SwerveModifier.forSpeed(0.25))
                .bindModifierToggle(buttonBox.button(2), SwerveModifier.forSpeed(0.1, 0.15))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.UP),
                    SwerveModifier.forCenterOffset(0.5, 0))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.DOWN),
                    SwerveModifier.forCenterOffset(-0.5, 0))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.RIGHT),
                    SwerveModifier.forCenterOffset(0, -0.5))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.LEFT),
                    SwerveModifier.forCenterOffset(0, 0.5)));

    driverJoystick
        .rightTrigger(0.75)
        .toggleOnTrue(new LockedSwerveDrive(swerve, LockedMode.XShape));

    DoneCycleCommand<VisionAlignToTarget> visionAlign =
        new DoneCycleCommand<>(
                new VisionAlignToTarget(
                    swerve,
                    limelight,
                    m_gyro,
                    DriveTrainConstants.SWERVE_ROTATION_SPEED,
                    () -> {
                      Pair<Double, Double> driveValues = getDriverLeftStickInput().get();
                      return driveValues.getFirst() == 0 && driveValues.getSecond() == 0;
                    }),
                true)
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(limelight::hasTargets, 10)
                    .withName("Has targets"))
            .withDoneCycles(
                cmd ->
                    DoneCycleMachine.supplierWithMinCycles(
                            cmd::pidAtSetpoint, VisionAlignToTarget.MIN_AT_SETPOINT_CYCLES)
                        .withName("At setpoint"));

    driverJoystick.y().toggleOnTrue(visionAlign);

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve, m_gyro, ApproachBehavior.fromVelocity(swerve, () -> 1.8)));
  }

  private Supplier<Pair<Double, Double>> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);
    root.addWidget(new CameraWidget("{11}:5800", true));
    root.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));
  }
}
