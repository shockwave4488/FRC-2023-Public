package frc.robot.robotspecifics.eruption;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import frc.lib.misc.Util;
import frc.lib.operator.CircularDeadzone;
import frc.lib.operator.I2DDeadzoneCalculator;
import frc.lib.operator.IDeadzoneCalculator;
import frc.lib.operator.SquareDeadzoneCalculator;
import frc.lib.preferences.PreferencesParser;
import frc.lib.preferences.PreferencesUtil;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.robot.Robot;
import frc.robot.autonomous.modes.eruption.AutonomousChooser;
import frc.robot.autonomous.modes.eruption.AutonomousTrajectories;
import frc.robot.commands.LEDs.SetLEDMode;
import frc.robot.commands.drive.HeadingRotation;
import frc.robot.commands.drive.RotateTowardsPosition;
import frc.robot.commands.drive.StandardDrive;
import frc.robot.commands.drive.StandardRotation;
import frc.robot.commands.eruption.defaults.DefaultDemoShooter;
import frc.robot.commands.eruption.defaults.DefaultIndexerLoad;
import frc.robot.commands.eruption.defaults.DefaultIntakeRetracted;
import frc.robot.commands.eruption.defaults.DefaultShooter;
import frc.robot.commands.eruption.drive.TurnToHubPoseThenVision;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.eruption.intake.ColorIntake;
import frc.robot.commands.eruption.intake.ColorlessIntake;
import frc.robot.commands.eruption.intake.PurgeAllIntake;
import frc.robot.commands.eruption.intake.PurgeBack;
import frc.robot.commands.eruption.shooter.CalculatedShot;
import frc.robot.commands.eruption.shooter.LaunchFenderShot;
import frc.robot.commands.eruption.shooter.PurgeForward;
import frc.robot.commands.eruption.shooter.SpinFlywheel;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants2022.FieldConstants;
import frc.robot.constants.Constants2022.ShooterConstants;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.Shooter;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.leds.LEDMode;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class EruptionRobotContainer extends BaseRobotContainer
    implements IDashboardSupportedContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final I2DDeadzoneCalculator bigCircularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final CommandXboxController driverJoystick;
  private final CommandGenericHID buttonBox;
  private final ISwerveModule[] swerveModules;
  private final Limelight shooterLimelight;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Indexer.StateSupplier indexerStates;
  private final Intake intake;
  private final LEDController ledController;
  private final SmartPCM smartPCM;
  private final AutonomousChooser autonomousChooser;
  private final AutoPIDControllerContainer pidControllers;
  private boolean demoMode;

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        PreferencesUtil.toObj(limelightPositionJSON, JSONPosition.class).toTransform());
  }

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public EruptionRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    bigCircularDeadzone = new CircularDeadzone(OIConstants.BIG_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);

    SwerveParameters[] swerveParameters =
        SwerveParameters.getAllFromJSONObject(prefs.getJSONObject("SwerveParameters"));
    swerveModules = new ISwerveModule[swerveParameters.length];

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i] = new SwerveModuleFalcons(swerveParameters[i], logger, prefs);
    }

    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    shooterLimelight =
        new Limelight(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);

    shooter =
        new Shooter(
            prefs.getInt("ShooterMFlywheelID"),
            prefs.getInt("ShooterFFlywheelID"),
            prefs.getInt("Shooter_PCM_ID"),
            prefs.getInt("HoodServo1ID"),
            prefs.getInt("HoodServo2ID"),
            logger,
            prefs);
    indexer =
        new Indexer(
            prefs.getInt("ConveyorID"),
            prefs.getInt("EntranceBBID"),
            prefs.getInt("MiddleBBID"),
            prefs.getInt("FlywheelBBID"),
            prefs);
    indexerStates = indexer.getIndexerStates();
    intake =
        new Intake(
            prefs.getInt("IntakeTopRollerID"),
            prefs.getInt("IntakeBottomRollerID"),
            prefs.getInt("Intake_PCM_ID"),
            prefs.getInt("IntakePistonID"),
            prefs.getInt("IntakeColorSensorID"),
            prefs.getInt("IntakeBallSensorID"));
    smartPCM = new SmartPCM(prefs.getInt("Intake_PCM_ID"));
    ledController = new LEDController(LEDMode.seismic(), prefs.getIntArray("LedDIO"));
    swerve =
        new SwerveDrive(
            m_gyro, Rotation2d.fromDegrees(prefs.getDouble("GyroAdjustment")), swerveModules);

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

    demoMode = prefs.tryGetValue(prefs::getBoolean, "DemoMode", true);

    /*
    Set initial pose to have the robot pointing at the HUB so the robot doesn't snap upon
    enabling This will be overridden in real matches because autonomousInit will run in Robot.java.
    */
    swerve.resetOdometry(new Pose2d(0, 4.15, new Rotation2d(0)));
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);

    swerve.driveRequirement.setDefaultCommand(
        new StandardDrive(
            swerve,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            getDriverLeftStickInput(),
            driverJoystick.start()));
    swerve.modifierRequirement.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(swerve::clearModifier, swerve.modifierRequirement)
            .withName("ResetSwerveModifier"));
    intake.setDefaultCommand(new DefaultIntakeRetracted(intake));
    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));
    indexer.setDefaultCommand(new DefaultIndexerLoad(indexer));

    // Assigns buttons bindings, default drive, and default shooter
    if (demoMode) {
      swerve.rotationRequirement.setDefaultCommand(getNewHeadingSwerveDriveCommand());
      shooter.setDefaultCommand(new DefaultDemoShooter(shooter));
      configureDemoModeBindings();
    } else {
      swerve.rotationRequirement.setDefaultCommand(
          new TurnToHubPoseThenVision(
              getNewSwerveTurnToHubCommand(), getNewVisionAlignToTargetCommand(false, 10)));
      shooter.setDefaultCommand(
          new DefaultShooter(
              shooter, shooterLimelight, indexerStates::getFlywheelBeamBreak, swerve::getOdometry));
      configureCompetitionBindings();
    }

    autonomousChooser =
        new AutonomousChooser(
            new AutonomousTrajectories(swerve, logger),
            swerve,
            intake,
            shooter,
            indexer,
            smartPCM,
            m_gyro,
            shooterLimelight,
            logger,
            prefs);

    addSubsystems();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(shooterLimelight);
    subsystems.add(shooter);
    subsystems.add(indexer);
    subsystems.add(intake);
    subsystems.add(ledController);
    subsystems.add(smartPCM);
  }

  protected void configureDemoModeBindings() {
    driverJoystick
        .back()
        .toggleOnTrue(
            new StandardRotation(
                    swerve,
                    -DriveTrainConstants.SWERVE_ROTATION_SPEED,
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.rightBumper().whileTrue(new PurgeAllIntake(intake, indexer, shooter));
    driverJoystick.leftBumper().whileTrue(new PurgeBack(intake, indexer, shooter));
    driverJoystick.rightTrigger(0.5).whileTrue(new ColorlessIntake(intake));
    driverJoystick.leftTrigger(0.5).whileTrue(new PurgeForward(indexer, shooter));
    driverJoystick.a().whileTrue(new ColorIntake(intake, indexerStates, false));
    driverJoystick.b().whileTrue(new ColorIntake(intake, indexerStates, true));
    driverJoystick.x().toggleOnTrue(new SetLEDMode(ledController, LEDMode.seismic()));

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileTrue(new RepeatCommand(getNewHeadingSwerveDriveCommand(false)));
  }

  protected void configureCompetitionBindings() {
    driverJoystick
        .back()
        .toggleOnTrue(
            new StandardRotation(
                    swerve,
                    -DriveTrainConstants.SWERVE_ROTATION_SPEED,
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.x().onTrue(getNewSwerveTurnToHubCommand());
    driverJoystick.leftTrigger(0.5).whileTrue(new PurgeBack(intake, indexer, shooter));
    driverJoystick.rightTrigger(0.5).toggleOnTrue(new ColorlessIntake(intake));
    driverJoystick
        .rightBumper()
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.leftBumper().whileTrue(new PurgeForward(indexer, shooter));

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve, m_gyro, ApproachBehavior.fromVelocity(swerve, () -> 1.8)));

    driverJoystick
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    driverJoystick
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));

    // Button box/operator bindings
    buttonBox.button(1).toggleOnTrue(getNewVisionAlignToTargetCommand(false, 10));
    // Toggle so operator can stop if aligned to the wrong target
    buttonBox
        .button(3)
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    buttonBox.button(4).onTrue(new InstantCommand(() -> shooterLimelight.takeSnapshot()));
    buttonBox
        .button(5)
        .onTrue(
            new LaunchFenderShot(
                shooter,
                indexer,
                smartPCM,
                ShooterConstants.FENDER_RPM,
                ShooterConstants.FENDER_HOOD_INPUT));
    Command chargeShooter =
        new DoneCycleCommand<>(
                new SpinFlywheel(
                    shooter,
                    shooterLimelight,
                    indexerStates::getFlywheelBeamBreak,
                    swerve::getOdometry,
                    true,
                    true),
                false)
            .withDoneCycles(DoneCycleMachine.fromSupplier(shooter::hoodReady))
            .withDoneCycles(shooter.flywheelVelocityMachine);
    buttonBox.button(6).whileTrue(chargeShooter);
    buttonBox
        .button(7)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));
    buttonBox
        .button(8)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    buttonBox.button(10).whileTrue(new PurgeBack(intake, indexer, shooter));
    buttonBox.button(11).whileTrue(new PurgeForward(indexer, shooter));
    buttonBox.button(14).whileTrue(new ColorIntake(intake, indexerStates, false));
    buttonBox.button(15).whileTrue(new ColorlessIntake(intake));
    buttonBox.button(16).whileTrue(new ColorIntake(intake, indexerStates, true));

    if (LeveledSmartDashboard.INFO.isEnabled()) {
      LeveledSmartDashboard.INFO.putData(
          "Limelight Snapshot",
          new InstantCommand(() -> shooterLimelight.takeSnapshot()).withName("Take snapshot"));
    }

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileTrue(new RepeatCommand(getNewHeadingSwerveDriveCommand(false)));
  }

  private Supplier<Pair<Double, Double>> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  private Supplier<Pair<Double, Double>> getDriverRightStickInput() {
    return () ->
        bigCircularDeadzone.deadzone(
            driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1);
  }

  private HeadingRotation getNewHeadingSwerveDriveCommand(boolean targetTakeover) {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new HeadingRotation(
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
        getDriverRightStickInput(),
        true);
  }

  private DoneCycleCommand<VisionAlignToTarget> getNewVisionAlignToTargetCommand(
      boolean stop, int hasTargetCycles) {
    return new DoneCycleCommand<>(
            new VisionAlignToTarget(
                swerve,
                shooterLimelight,
                m_gyro,
                DriveTrainConstants.SWERVE_ROTATION_SPEED,
                () -> {
                  Pair<Double, Double> driveValues = getDriverLeftStickInput().get();
                  return driveValues.getFirst() == 0 && driveValues.getSecond() == 0;
                }),
            stop)
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(shooterLimelight::hasTargets, hasTargetCycles)
                .withName("Has targets"))
        .withDoneCycles(
            cmd ->
                DoneCycleMachine.supplierWithMinCycles(
                        cmd::pidAtSetpoint, VisionAlignToTarget.MIN_AT_SETPOINT_CYCLES)
                    .withName("At yaw setpoint"));
  }

  private HeadingRotation getNewHeadingSwerveDriveCommand() {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new HeadingRotation(
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
        getDriverRightStickInput(),
        true);
  }

  public RotateTowardsPosition getNewSwerveTurnToHubCommand() {
    return Util.returnAfterModifying(
        new RotateTowardsPosition(
            swerve, m_gyro, pidControllers.thetaPidController, () -> FieldConstants.HUB_CENTER),
        cmd -> cmd.setName("TurnToHub"));
  }

  private CalculatedShot getNewCalculatedShotCommand() {
    return new CalculatedShot(
        shooter,
        indexer,
        smartPCM,
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_ROTATION_SPEED,
        shooterLimelight,
        getNewSwerveTurnToHubCommand(),
        getNewHeadingSwerveDriveCommand(true),
        getNewVisionAlignToTargetCommand(false, CalculatedShot.MIN_HAS_TARGET_CYCLES));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);
    root.addWidget(new CameraWidget("{11}:5800", true));
    root.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2022));
  }
}
