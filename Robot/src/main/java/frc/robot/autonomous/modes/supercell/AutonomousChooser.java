package frc.robot.autonomous.modes.supercell;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.LogCommand;
import frc.lib.geometry.PoseUtil;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.robot.commands.drive.SwerveDriveToPosition;
import frc.robot.commands.other.InitPositionFromTag;
import frc.robot.commands.supercell.AutoScoreCommandBuilder;
import frc.robot.commands.supercell.arm.MoveArmWithPID;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc.robot.commands.supercell.intake.IntakeCommand;
import frc.robot.constants.Constants2023.FieldConstants;
import frc.robot.constants.Constants2023.GamePiece;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc.robot.constants.Constants2023.RobotConstants.AutonomousConstants;
import frc.robot.constants.Constants2023.ScoreLevel;
import frc.robot.constants.Constants2023.ScorePosition;
import frc.robot.robotspecifics.supercell.SelectedConstants;
import frc.robot.robotspecifics.supercell.SupercellRobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.supercell.Arm;
import frc.robot.subsystems.supercell.Intake;
import frc.robot.subsystems.supercell.Intake.Speed;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

public class AutonomousChooser {

  private final SwerveDrive swerve;
  private final Arm arm;
  private final Intake intake;
  private final Limelight camera;
  private final NavX gyro;
  private final AutoPIDControllerContainer pid;
  private final Supplier<TrajectoryConfig> config;
  private final SelectedConstants selectedConstants;
  private final SwerveAutoBuilder autoBuilder;
  private final PreferencesParser prefs;
  private final LogManager logger;

  public AutonomousChooser(
      SwerveDrive swerve,
      Arm arm,
      Intake intake,
      Limelight camera,
      NavX gyro,
      AutoPIDControllerContainer pid,
      Supplier<TrajectoryConfig> config,
      SelectedConstants selectedConstants,
      PreferencesParser prefs,
      LogManager logger) {
    this.swerve = swerve;
    this.arm = arm;
    this.intake = intake;
    this.camera = camera;
    this.gyro = gyro;
    this.pid = pid;
    this.config = config;
    this.selectedConstants = selectedConstants;
    this.prefs = prefs;
    this.logger = logger;

    HashMap<String, Command> eventMap = new HashMap<>();

    autoBuilder =
        new SwerveAutoBuilder(
            swerve::getOdometry,
            swerve::resetOdometry,
            swerve.getKinematics(),
            new PIDConstants(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDConstants(prefs.getDouble("AutoTurnP"), 0.0, 0.0),
            swerve::assignModuleStates,
            eventMap,
            true,
            swerve.driveRequirement,
            swerve.rotationRequirement);
  }

  public enum AutonomousMode {
    PATH_PLANNER_CURVE("Path Planner Curve Test"),
    PATH_PLANNER("Path Planner Test"),
    SCORE_AND_BALANCE("Score And Balance"),
    SCORE_AND_PICKUP_TOP("Score And Pickup Top"),
    SCORE_AND_PICKUP_BOTTOM("Score And Pickup Bottom"),
    SCORE_AND_DRIVE_OVER_CHARGE_STATION("Score And Drive Over Charge Station"),
    SCORE_AND_LEAVE_COMMUNITY_TOP("Score And Leave Community Top"),
    SCORE_AND_LEAVE_COMMUNITY_BOTTOM("Score And Leave Community Bottom"),
    SCORE_CENTER("Score Center And BackUp"),
    SCORE_CUBE_AND_BALANCE("Score Cube And Balance"),
    DO_NOTHING("Do Nothing");

    private final String name;

    private AutonomousMode(String name) {
      this.name = name;
    }

    public String getNiceName() {
      return name;
    }
  }

  public CommandBase getDoNothingCommand() {
    return new InstantCommand();
  }

  public CommandBase getAutoScoreCommand(ScoreLevel level, ScorePosition position) {
    return LogCommand.proxy(
        AutoScoreCommandBuilder.createForAnyTag(
                level,
                position,
                swerve,
                gyro,
                intake,
                pid,
                camera,
                () -> true,
                () -> true,
                () -> false)
            .withArm(arm));
  }

  public Command getMoveToCommand(Supplier<Pose2d> pos, boolean reversed) {
    return new SwerveDriveToPosition(
        swerve, pid, () -> config.get().setReversed(reversed), pos, () -> new Translation2d[0]);
  }

  public CommandBase getScoreAndBackUpCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(ScoreLevel.HIGH, ScorePosition.CONE_LEFT),
        LogCommand.proxy(
            getMoveToCommand(
                () ->
                    swerve
                        .getOdometry()
                        .plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d())),
                true)));
  }

  public CommandBase getScoreAndLeaveCommunityCommand(double timeoutSeconds, boolean top) {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(
            ScoreLevel.HIGH, top ? ScorePosition.CONE_RIGHT : ScorePosition.CONE_LEFT),
        LogCommand.proxy(
            LogCommand.endAfter(
                new RunCommand(
                    () -> swerve.setTranslationSpeeds(2, 0, true), swerve.driveRequirement),
                timeoutSeconds)));
  }

  public CommandBase getScoreAndBalanceCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(ScoreLevel.HIGH, ScorePosition.CONE_RIGHT),
        LogCommand.proxy(
            DriveAndBalanceOnChargeStation.create(
                swerve,
                gyro,
                ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                    .withDelayTime(swerve, gyro, 1.8, 1)
                    .withTimeout(4))));
  }

  public CommandBase getScoreCubeAndBalanceCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        LogCommand.proxy(
            LogCommand.sequence(
                LogCommand.endAfter(IntakeCommand.launchCube(intake), 0.5),
                new InstantCommand(() -> intake.setSpeed(Speed.STOPPED), intake),
                DriveAndBalanceOnChargeStation.create(
                    swerve,
                    gyro,
                    ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                        .withDelayTime(swerve, gyro, 1.8, 1))))); // TODO: add 4s timeout
  }

  public CommandBase getScoreAndPickupCommand(boolean top) {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, top ? 3.91 : 0.48, new Rotation2d(Math.PI))),
        getAutoScoreCommand(
            ScoreLevel.HIGH, top ? ScorePosition.CONE_RIGHT : ScorePosition.CONE_LEFT),
        getMoveToCommand(
            () -> PoseUtil.addTranslation(swerve.getOdometry(), new Translation2d(1, 0)), true),
        LogCommand.proxy(
            new SwerveDriveToPosition(
                swerve,
                pid,
                () -> config.get().setReversed(true),
                () -> PoseUtil.addTranslation(swerve.getOdometry(), new Translation2d(1, 0)),
                () -> new Translation2d[0])),
        LogCommand.proxy(
            LogCommand.sequence(
                LogCommand.parallel(
                    MoveArmWithPID.createForAuto(arm, ArmSetpoint.PIECE_PICKUP),
                    getMoveToCommand(
                        () ->
                            new Pose2d(
                                FieldConstants.PRESET_PIECE_X - 1.5,
                                FieldConstants.getInstance()
                                    .presetPiecePositions
                                    .get(top ? 1 : 4)
                                    .getY(),
                                new Rotation2d()),
                        false)),
                LogCommand.race(
                    LogCommand.proxy(
                        selectedConstants.gamePieceChoosers.get(top ? 1 : 4).getSelected()
                                == GamePiece.Cube
                            ? IntakeCommand.in(intake, GamePiece.Cube)
                            : SupercellRobotContainer.intakeConeWithHold(intake, prefs)),
                    getMoveToCommand(
                        () ->
                            new Pose2d(
                                FieldConstants.PRESET_PIECE_X,
                                FieldConstants.getInstance()
                                    .presetPiecePositions
                                    .get(top ? 1 : 4)
                                    .getY(),
                                new Rotation2d()),
                        false)))));
  }

  public CommandBase getPathPlannerTestCommand() {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup(
            "LeaveCommunityAndDock",
            new PathConstraints(
                AutonomousConstants.AUTO_MAX_VELOCITY, AutonomousConstants.AUTO_MAX_ACCELERATION));
    Command fullTestAuto = autoBuilder.fullAuto(pathGroup);

    logger.getMainLog().println(LogLevel.INFO, "LeaveCommunityAndDock");

    return resetRobotPose(pathGroup.get(0).getInitialPose()).andThen(fullTestAuto);
  }

  public CommandBase getPathPlannerCurveTestCommand() {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup(
            "CurvySpinnyPathyThing",
            new PathConstraints(
                AutonomousConstants.AUTO_MAX_VELOCITY, AutonomousConstants.AUTO_MAX_ACCELERATION));
    Command fullCurveTestAuto = autoBuilder.fullAuto(pathGroup);

    logger.getMainLog().println(LogLevel.INFO, "CurvySpinnyThing");

    return resetRobotPose(pathGroup.get(0).getInitialPose()).andThen(fullCurveTestAuto);
  }

  public Command getCommand(AutonomousMode mode) {
    if (mode == null) {
      return getDoNothingCommand();
    }

    CommandBase autoCommand =
        switch (mode) {
          case PATH_PLANNER:
            yield getPathPlannerTestCommand();
          case PATH_PLANNER_CURVE:
            yield getPathPlannerCurveTestCommand();
          case SCORE_AND_BALANCE:
            yield getScoreAndBalanceCommand();
          case SCORE_AND_PICKUP_TOP:
            yield getScoreAndPickupCommand(true);
          case SCORE_AND_PICKUP_BOTTOM:
            yield getScoreAndPickupCommand(false);
          case SCORE_AND_DRIVE_OVER_CHARGE_STATION:
            yield getScoreAndLeaveCommunityCommand(2.5, false);
          case SCORE_AND_LEAVE_COMMUNITY_TOP:
            yield getScoreAndLeaveCommunityCommand(2.0, true);
          case SCORE_AND_LEAVE_COMMUNITY_BOTTOM:
            yield getScoreAndLeaveCommunityCommand(2.0, false);
          case SCORE_CENTER:
            yield getScoreAndBackUpCommand();
          case SCORE_CUBE_AND_BALANCE:
            yield getScoreCubeAndBalanceCommand();
          default:
            yield getDoNothingCommand();
        };
    autoCommand.setName(mode.getNiceName().replaceAll("\\s+", ""));
    return autoCommand;
  }

  private Command resetRobotPose(Pose2d pose) {
    return new InstantCommand(
        () -> {
          gyro.setYawAdjustment(new Rotation2d());
          gyro.setYawAdjustment(pose.getRotation().minus(gyro.getYaw()));
          swerve.resetOdometry(pose);
        });
  }

  private Command resetRobotPoseFromTag(Pose2d fallback) {
    return LogCommand.race(
        InitPositionFromTag.create(swerve, gyro, camera),
        new WaitCommand(0.5).andThen(resetRobotPose(fallback)));
  }
}
