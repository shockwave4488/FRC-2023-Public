package frc.robot.robotspecifics.supercell;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BaseRobotContainer;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.CommandUtil;
import frc.lib.commands.LogCommand;
import frc.lib.dashboard.DashboardServer;
import frc.lib.dashboard.IDashboardSupportedContainer;
import frc.lib.dashboard.gui.ButtonWidget;
import frc.lib.dashboard.gui.CameraWidget;
import frc.lib.dashboard.gui.DropdownWidget;
import frc.lib.dashboard.gui.FieldWidget;
import frc.lib.dashboard.gui.GroupWidget;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.LogManager;
import frc.lib.math.JSONPosition;
import frc.lib.misc.MatchUtil;
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
import frc.robot.autonomous.modes.supercell.AutonomousChooser;
import frc.robot.commands.LEDs.SetLEDMode;
import frc.robot.commands.drive.HeadingRotation;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc.robot.commands.drive.StandardDrive;
import frc.robot.commands.drive.StandardRotation;
import frc.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc.robot.commands.supercell.AutoScoreCommandBuilder;
import frc.robot.commands.supercell.arm.MoveArmWithPID;
import frc.robot.commands.supercell.drive.BalanceOnChargeStation;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc.robot.commands.supercell.intake.HoldCone;
import frc.robot.commands.supercell.intake.IntakeCommand;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants2023.FieldConstants;
import frc.robot.constants.Constants2023.GamePiece;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.leds.LEDMode;
import frc.robot.subsystems.supercell.Arm;
import frc.robot.subsystems.supercell.Intake;
import frc.robot.subsystems.supercell.Intake.Speed;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SupercellRobotContainer extends BaseRobotContainer
    implements IDashboardSupportedContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final Intake intake;
  private final Arm arm;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final I2DDeadzoneCalculator bigCircularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final CommandXboxController driverJoystick;
  private final CommandGenericHID buttonBox;
  private final ISwerveModule[] swerveModules;
  private final Limelight limelight;
  private final SmartPCM smartPCM;
  private final LEDController ledController;
  private final AutonomousChooser autonomousChooser;
  private final AutoPIDControllerContainer autoPidControllers;
  private final SelectedConstants selectedConstants;

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        PreferencesUtil.toObj(limelightPositionJSON, JSONPosition.class).toTransform());
  }

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public SupercellRobotContainer(PreferencesParser prefs, LogManager logger) {
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

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));
    ledController = new LEDController(LEDMode.seismic(), prefs.getIntArray("LedDIO"));

    selectedConstants = new SelectedConstants();

    intake = new Intake(prefs.getInt("IntakeRollerID"), prefs.getInt("IntakeTimeOfFlightID"));
    swerve =
        new SwerveDrive(
            m_gyro, Rotation2d.fromDegrees(prefs.getDouble("GyroAdjustment")), swerveModules);
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);
    arm = new Arm(prefs, logger);

    autoPidControllers =
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
                0,
                0,
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI)));

    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    limelight =
        new Limelight(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);

    autonomousChooser =
        new AutonomousChooser(
            swerve,
            arm,
            intake,
            limelight,
            m_gyro,
            autoPidControllers,
            () ->
                new TrajectoryConfig(
                    DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                    DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL),
            selectedConstants,
            prefs,
            logger);

    swerve.driveRequirement.setDefaultCommand(
        new StandardDrive(
            swerve,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            getDriverLeftStickInput(),
            driverJoystick.start()));
    swerve.rotationRequirement.setDefaultCommand(
        getNewHeadingSwerveDriveCommand(DriveTrainConstants.SWERVE_ROTATION_SPEED * -1));
    swerve.modifierRequirement.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(swerve::clearModifier, swerve.modifierRequirement)
            .withName("ResetSwerveModifier"));

    smartPCM.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(smartPCM::startCompressor, smartPCM)
            .withName("CompressorCommand"));
    arm.setDefaultCommand(
        new MoveArmWithPID(arm, arm.getMinimumSetpoint()).withName("ArmDefaultCommand"));
    intake.setDefaultCommand(
        new IntakeCommand(intake, Speed.STOPPED).withName("IntakeDefaultCommand"));

    ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");

    if (!MatchUtil.isSimulated()) {
      UsbCamera usbCamera = CameraServer.startAutomaticCapture();
      usbCamera.setResolution(320, 180);
      if (RobotBase.isReal()) {
        usbCamera.setFPS(15);
      }
      competitionTab.add("Front camera", usbCamera).withSize(3, 3).withPosition(6, 2);
    }

    ShuffleboardLayout gyroResetGrid =
        competitionTab
            .getLayout("Gyro reset", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1))
            .withSize(3, 1)
            .withPosition(0, 3);
    GenericEntry resetYawEntry = gyroResetGrid.add("Yaw (degrees)", 0).getEntry("double");
    gyroResetGrid.add(
        "Reset gyro",
        new InstantCommand(
                () -> {
                  m_gyro.reset();
                  m_gyro.setYawAdjustment(Rotation2d.fromDegrees(resetYawEntry.getDouble(0)));
                  swerve.resetOdometry(
                      new Pose2d(swerve.getOdometry().getTranslation(), m_gyro.getYaw()));
                })
            .ignoringDisable(true)
            .withName("ResetGyro"));

    FieldConstants.eagerlyInitialize();

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
    subsystems.add(smartPCM);
    subsystems.add(intake);
    subsystems.add(arm);
    subsystems.add(ledController);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    driverJoystick.rightTrigger().toggleOnTrue(new LockedSwerveDrive(swerve, LockedMode.XShape));

    driverJoystick
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> swerve.setModifier(SwerveModifier.forSpeed(0.5)),
                swerve::clearModifier,
                swerve.modifierRequirement));

    driverJoystick
        .rightBumper()
        .whileTrue(
            new StandardRotation(
                    swerve,
                    -DriveTrainConstants.SWERVE_ROTATION_SPEED,
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                .alongWith(
                    new StartEndCommand(
                        () -> swerve.setModifier(SwerveModifier.forCenterOffset(0.5, 0)),
                        swerve::clearModifier,
                        swerve.modifierRequirement)));

    driverJoystick
        .start()
        .toggleOnTrue(
            new StandardRotation(
                swerve,
                -DriveTrainConstants.SWERVE_ROTATION_SPEED,
                () -> squareDeadzone.deadzone(driverJoystick.getRightX())));

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve,
                m_gyro,
                ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                    .withDelayTime(swerve, m_gyro, 2, 1.2)
                    .withRotation(
                        swerve, m_gyro, autoPidControllers.thetaPidController, new Rotation2d())));
    driverJoystick.b().whileTrue(BalanceOnChargeStation.create(swerve, m_gyro));

    // Arm Bindings
    buttonBox.button(1).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.PIECE_PICKUP));
    buttonBox.button(16).onTrue(new InstantCommand(() -> arm.raiseOffset()));
    buttonBox.button(15).onTrue(new InstantCommand(() -> arm.lowerOffset()));
    // Intake Bindings
    final Trigger CONE_PURGE_BUTTON = buttonBox.button(13);
    final Trigger CUBE_PURGE_BUTTON = buttonBox.button(12);
    final Trigger CONE_INTAKE_BUTTON = buttonBox.button(9);
    final Trigger CUBE_INTAKE_BUTTON = buttonBox.button(8);
    CONE_INTAKE_BUTTON.whileTrue(intakeConeWithHold(intake, prefs));
    CONE_INTAKE_BUTTON.whileTrue(new SetLEDMode(ledController, LEDMode.solid(0xFF6600)));
    CONE_PURGE_BUTTON.whileTrue(IntakeCommand.out(intake, GamePiece.Cone));
    CUBE_INTAKE_BUTTON.whileTrue(IntakeCommand.in(intake, GamePiece.Cube).toDoneCycleCommand());
    CUBE_INTAKE_BUTTON.whileTrue(new SetLEDMode(ledController, LEDMode.solid(0x9600E1)));
    CUBE_PURGE_BUTTON.whileTrue(IntakeCommand.out(intake, GamePiece.Cube));
    driverJoystick.y().whileTrue(IntakeCommand.launchCube(intake));
    buttonBox.button(14).whileTrue(IntakeCommand.launchCube(intake));

    // bindManualScoringCommands();
    bindAutoScoringCommands(
        driverJoystick.leftBumper().or(CONE_PURGE_BUTTON).or(CUBE_PURGE_BUTTON));

    // buttonBox
    //     .button(2)
    //     .whileTrue(
    //         LogCommand.endWhen(
    //             AlignToSubstation.from(
    //                 swerve,
    //                 m_gyro,
    //                 limelight,
    //                 arm,
    //                 autoPidControllers,
    //                 selectedConstants.substationSideChooser::getSelected),
    //             driverJoystick.rightBumper()));
    buttonBox.button(2).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.SUBSTATION));
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    GroupWidget root = dashboard.getWebsite().getWidgets();

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    root.addWidget(leftGroup);
    root.addWidget(rightGroup);

    leftGroup.addWidget(new CameraWidget("{11}:5800", true));
    leftGroup.addWidget(new CameraWidget("{2}:1181", true));

    rightGroup.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));

    GroupWidget options = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    rightGroup.addWidget(options);

    options.addWidget(
        new ButtonWidget(
            "Reset Gyro",
            dashboard,
            () -> {
              m_gyro.reset();
              m_gyro.setYawAdjustment(Rotation2d.fromDegrees(0));
              swerve.resetOdometry(
                  new Pose2d(swerve.getOdometry().getTranslation(), m_gyro.getYaw()));
            }));

    options.addWidget(
        new DropdownWidget<>(
            "Auto mode", "/Shuffleboard/Competition/Auto mode", selectedConstants.autoModeChooser));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 1",
            "/Shuffleboard/Competition/Game piece 1",
            selectedConstants.gamePieceChoosers.get(1)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 2",
            "/Shuffleboard/Competition/Game piece 2",
            selectedConstants.gamePieceChoosers.get(2)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 3",
            "/Shuffleboard/Competition/Game piece 3",
            selectedConstants.gamePieceChoosers.get(3)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 4",
            "/Shuffleboard/Competition/Game piece 4",
            selectedConstants.gamePieceChoosers.get(4)));
    options.addWidget(
        new DropdownWidget<>(
            "Start game piece",
            "/Shuffleboard/Competition/Start game piece",
            selectedConstants.startPieceChooser));
    options.addWidget(
        new DropdownWidget<>(
            "Substation side",
            "/Shuffleboard/Competition/Substation side",
            selectedConstants.substationSideChooser));
  }

  public static Command intakeConeWithHold(Intake intake, PreferencesParser prefs) {
    return LogCommand.sequence(
        IntakeCommand.in(intake, GamePiece.Cone).toDoneCycleCommand(),
        new ScheduleCommand(HoldCone.basedOnPrefs(intake, prefs)));
  }

  private void bindManualScoringCommands() {
    buttonBox.button(3).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.LOW_SCORE));
    buttonBox.button(4).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.MID_SCORE));
    buttonBox.button(5).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.HIGH_SCORE));
  }

  private void bindAutoScoringCommands(BooleanSupplier shouldCancel) {
    buttonBox.button(5).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.LOW_SCORE));
    AutoScoreCommandBuilder.bindToButtonBox(
        swerve,
        m_gyro,
        intake,
        autoPidControllers,
        limelight,
        builder -> builder.withArm(arm),
        buttonBox,
        shouldCancel,
        new int[][] {new int[] {11, 7, 4}, new int[] {10, 6, 3}, new int[] {0, 0, 0}});
  }

  private Supplier<Pair<Double, Double>> getDriverRightStickInput() {
    return () ->
        bigCircularDeadzone.deadzone(
            driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1);
  }

  private Supplier<Pair<Double, Double>> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  private HeadingRotation getNewHeadingSwerveDriveCommand(double rotationMultiplier) {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new HeadingRotation(
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_ROTATION_SPEED * rotationMultiplier,
        getDriverRightStickInput(),
        false);
  }

  public Command getAutonomousCommand() {
    return LogCommand.sequence(
        new ScheduleCommand(HoldCone.basedOnPrefs(intake, prefs))
            .unless(() -> selectedConstants.startPieceChooser.getSelected() != GamePiece.Cone),
        autonomousChooser.getCommand(selectedConstants.autoModeChooser.getSelected()));
  }
}
