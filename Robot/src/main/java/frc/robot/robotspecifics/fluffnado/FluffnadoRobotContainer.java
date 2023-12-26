package frc.robot.robotspecifics.fluffnado;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.BaseRobotContainer;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.CommandUtil;
import frc.lib.commands.LogCommand;
import frc.lib.dashboard.DashboardServer;
import frc.lib.dashboard.IDashboardSupportedContainer;
import frc.lib.dashboard.gui.ButtonWidget;
import frc.lib.dashboard.gui.CameraWidget;
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
import frc.robot.commands.drive.HeadingRotation;
import frc.robot.commands.drive.StandardDrive;
import frc.robot.commands.drive.StandardRotation;
import frc.robot.commands.drive.SwerveDriveToPosition;
import frc.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc.robot.commands.fluffnado.arm.MoveArm;
import frc.robot.commands.fluffnado.catapult.Launch;
import frc.robot.commands.fluffnado.catapult.ResetCatapult;
import frc.robot.commands.fluffnado.intake.IntakeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants2024.ArmConstants;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import frc.robot.subsystems.fluffnado.Arm;
import frc.robot.subsystems.fluffnado.Catapult;
import frc.robot.subsystems.fluffnado.Intake;
import frc.robot.subsystems.fluffnado.Intake.Speed;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

public class FluffnadoRobotContainer extends BaseRobotContainer
    implements IDashboardSupportedContainer {

  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final Catapult catapult;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final I2DDeadzoneCalculator bigCircularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final CommandXboxController driverJoystick;
  private final CommandGenericHID buttonBox;
  private final ISwerveModule[] swerveModules;
  private final Limelight limelight;
  private final Intake intake;
  private final Arm arm;

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        PreferencesUtil.toObj(limelightPositionJSON, JSONPosition.class).toTransform());
  }

  public FluffnadoRobotContainer(PreferencesParser prefs, LogManager logger) {
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

    swerve =
        new SwerveDrive(
            m_gyro, Rotation2d.fromDegrees(prefs.getDouble("GyroAdjustment")), swerveModules);
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);
    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    limelight =
        new Limelight(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);
    intake = new Intake(prefs.getInt("IntakeRollerID"));
    intake.setDefaultCommand(
        new IntakeCommand(intake, Speed.STOPPED).withName("IntakeDefaultCommand"));
    catapult =
        new Catapult(
            prefs,
            prefs.getInt("CatapultMainMotorID"),
            prefs.getInt("CatapultFollowMotorID"),
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

    arm = new Arm(prefs, logger);
    arm.setDefaultCommand(new MoveArm(arm, ArmConstants.STARTING).withName("MoveArmUp"));
    catapult.setDefaultCommand(new ResetCatapult(catapult));

    if (!MatchUtil.isSimulated()) {
      UsbCamera usbCamera = CameraServer.startAutomaticCapture();
      usbCamera.setResolution(320, 180);
      if (RobotBase.isReal()) {
        usbCamera.setFPS(15);
      }
    }

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
    subsystems.add(intake);
    subsystems.add(arm);
    subsystems.add(catapult);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

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
    // Arm commands
    buttonBox.button(6).whileTrue(new MoveArm(arm, ArmConstants.DOWN).withName("MoveArmDown"));
    buttonBox.button(7).whileTrue(new MoveArm(arm, ArmConstants.TOTE).withName("Arm Tote pickup"));
    buttonBox
        .button(8)
        .whileTrue(new MoveArm(arm, ArmConstants.DELIVERY).withName("Arm delivery point"));
    buttonBox.button(7).whileTrue(new MoveArm(arm, ArmConstants.DOWN).withName("MoveArmDown"));
    // Intake commands
    buttonBox.button(11).whileTrue(IntakeCommand.in(intake).withName("Intake In"));
    buttonBox.button(12).whileTrue(IntakeCommand.out(intake).withName("Intake out"));
    // Catapult commands
    buttonBox
        .button(9)
        .onTrue(getLaunchCatapultAndMoveArmCommand(10, 81.0, "Hard Launch 70 degrees", 0.33));
    buttonBox
        .button(5)
        .onTrue(getLaunchCatapultAndMoveArmCommand(12, 56.0, "Hard Launch 45 degrees", 0.33));
    buttonBox
        .button(4)
        .onTrue(getLaunchCatapultAndMoveArmCommand(8, 56.0, "Soft Launch 45 degrees", 0.33));
    buttonBox.button(3).onTrue(getLaunchCatapultAndMoveArmCommand(0.5, 101.0, "Purge", 3));
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

    rightGroup.addWidget(
        new FieldWidget("Field", FieldWidget.FieldYear.Y2023)); // Might want to make our own widget

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

  private Command getLaunchCatapultAndMoveArmCommand(
      double speed, double goalAngle, String launchName, double timeout) {
    Command command =
        LogCommand.sequence(
                new MoveArm(arm, ArmConstants.DOWN)
                    .withName("MoveArmDown")
                    .until(
                        () ->
                            arm.getMeasurement()
                                < ArmConstants.DOWN + Rotation2d.fromDegrees(5).getRadians()),
                new Launch(catapult, speed, goalAngle).withName(launchName).withTimeout(timeout),
                new ResetCatapult(catapult).until(() -> catapult.getAngle().getDegrees() < 5))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    return command;
  }

  @Override
  public Command getAutonomousCommand() {
    return new SwerveDriveToPosition(
        swerve,
        new AutoPIDControllerContainer(
            new PIDController(
                prefs.getDouble("SwerveFalconsDriveP"),
                prefs.getDouble("SwerveFalconsDriveI"),
                prefs.getDouble("SwerveFalconsDriveD")),
            new PIDController(
                prefs.getDouble("SwerveFalconsDriveP"),
                prefs.getDouble("SwerveFalconsDriveI"),
                prefs.getDouble("SwerveFalconsDriveD")),
            new ProfiledPIDController(
                prefs.getDouble("SwerveFalconsTurnP"),
                prefs.getDouble("SwerveFalconsTurnI"),
                prefs.getDouble("SwerveFalconsTurnD"),
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI))),
        () ->
            new TrajectoryConfig(
                Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL),
        () -> swerve.getOdometry().plus(new Transform2d(new Translation2d(3, 0), new Rotation2d())),
        () -> new Translation2d[0]);
  }
}
