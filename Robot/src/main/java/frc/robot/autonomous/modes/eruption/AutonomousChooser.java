package frc.robot.autonomous.modes.eruption;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.VisionCamera;
import frc.lib.sensors.vision.VisionCameras.TargetCamera;
import frc.robot.commands.drive.HeadingRotation;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.commands.drive.RotateTowardsPosition;
import frc.robot.commands.drive.SwerveDriveToPosition;
import frc.robot.commands.eruption.defaults.DefaultIndexerLoad;
import frc.robot.commands.eruption.drive.RotateToAngleAuto;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.eruption.intake.ColorIntake;
import frc.robot.commands.eruption.intake.ColorlessIntake;
import frc.robot.commands.eruption.intake.PurgeBack;
import frc.robot.commands.eruption.intake.PurgeIntake;
import frc.robot.commands.eruption.shooter.CalculatedShot;
import frc.robot.commands.eruption.shooter.LaunchSetShot;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants2022.AutonomousConstants;
import frc.robot.constants.Constants2022.FieldConstants;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.Shooter;
import java.util.function.Supplier;

public class AutonomousChooser {
  private final AutoPIDControllerContainer pidControllers;
  private final AutonomousTrajectories trajectories;
  private final SwerveDrive swerve;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private Indexer.StateSupplier indexerStates;
  private final SmartPCM smartPCM;
  private final NavX gyro;
  private final TargetCamera limelight;
  private final LogManager logger;
  private static final Pair<Double, Double> FAKE_INPUTS_DOUBLE = Pair.of(0., 0.);
  private static final Supplier<Pair<Double, Double>> FAKE_INPUTS_SUPPLIER =
      () -> FAKE_INPUTS_DOUBLE;

  private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
  private SendableChooser<IntakeMode> colorIntakeToggle = new SendableChooser<>();

  public AutonomousChooser(
      AutonomousTrajectories trajectories,
      SwerveDrive swerve,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      SmartPCM smartPCM,
      NavX gyro,
      TargetCamera limelight,
      LogManager logger,
      PreferencesParser prefs) {
    this.swerve = swerve;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    if (indexer != null) {
      indexerStates = indexer.getIndexerStates();
    }
    this.smartPCM = smartPCM;
    this.limelight = limelight;
    this.gyro = gyro;
    this.logger = logger;
    this.trajectories = trajectories;

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
                0,
                0,
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI)));

    autonomousModeChooser.addOption("One Ball Auto Mid", AutonomousMode.ONE_BALL_MID);
    autonomousModeChooser.addOption(
        "Mean Shoot 2, Hangar Ball Left", AutonomousMode.MEAN_SHOOT_TWO_BALL_HANGAR_BALL_LEFT);
    autonomousModeChooser.addOption(
        "Mean Shoot 2, Fender Ball Left", AutonomousMode.MEAN_SHOOT_TWO_BALL_FENDER_BALL_LEFT);
    autonomousModeChooser.addOption(
        "Mean Shoot 2, Steal Two Ball Left",
        AutonomousMode.MEAN_SHOOT_TWO_BALL_STEAL_TWO_BALL_LEFT);
    autonomousModeChooser.addOption("Three Ball Auto Right", AutonomousMode.THREE_BALL_RIGHT);
    autonomousModeChooser.setDefaultOption(
        "Five Ball Auto Waypoints", AutonomousMode.FIVE_BALL_RIGHT_WAYPOINTS);
    autonomousModeChooser.addOption("Back And Forth Auto", AutonomousMode.BACK_AND_FORTH_TEST);

    LeveledSmartDashboard.HIGH.putData(autonomousModeChooser);

    colorIntakeToggle.setDefaultOption("Color Intake (normal purge) Auto", IntakeMode.COLOR_NORMAL);
    colorIntakeToggle.addOption("Color Intake (hard purge) Auto", IntakeMode.COLOR_HARD);
    colorIntakeToggle.addOption("Colorless Intake Auto", IntakeMode.COLORLESS);
    LeveledSmartDashboard.HIGH.putData(colorIntakeToggle);
  }

  public enum AutonomousMode {
    ONE_BALL_MID,
    MEAN_SHOOT_TWO_BALL_HANGAR_BALL_LEFT,
    MEAN_SHOOT_TWO_BALL_FENDER_BALL_LEFT,
    MEAN_SHOOT_TWO_BALL_STEAL_TWO_BALL_LEFT,
    THREE_BALL_RIGHT,
    FIVE_BALL_RIGHT_WAYPOINTS,
    BACK_AND_FORTH_TEST,
  }

  private enum IntakeMode {
    COLOR_NORMAL,
    COLOR_HARD,
    COLORLESS,
    PURGE
  }

  public Command getOneBallMidCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    // resets the gyro
    resetRobotPose(command, trajectories.getOneBallMid());
    // continues along the trajectory while intaking
    followTrajectory(command, trajectories.getOneBallMid());
    // stops
    stopMoving(command);
    // starts shooting
    autoShot(command);
    // stops
    stopMoving(command);

    return command;
  }

  public Command getMeanShootTwoBallHangarBallLeftCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    // resets the gyro
    resetRobotPose(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // stops
    stopMoving(command);
    // follows second trajectory
    followTrajectoryAndIntake(command, trajectories.getMeanTwoBallLeftPartTwoTrajectory());
    // stops
    stopMoving(command);
    // starts shooting
    blindShotFromInsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_INSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC,
        2);
    // follows the second trajectory while intaking
    followTrajectoryAndIntakeForced(
        command, trajectories.getMeanTwoBallLeftPartThreeTrajectory(), IntakeMode.COLORLESS);
    // stops
    stopMoving(command);
    // rotates for the intake to face our hangar
    rotateToAngle(command, 0);
    // stops
    stopMoving(command);
    // purges towards the hangar
    purgeThroughIntake(command, 5);
    // stops
    stopMoving(command);

    return command;
  }

  public Command getMeanShootTwoBallFenderBallLeftCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    // resets the gyro
    resetRobotPose(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // stops
    stopMoving(command);
    // follows second trajectory
    followTrajectoryAndIntake(command, trajectories.getMeanTwoBallLeftPartTwoTrajectory());
    // stops
    stopMoving(command);
    // starts shooting
    blindShotFromInsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_INSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC,
        2);
    // follows the second trajectory while intaking
    followTrajectoryAndIntakeForced(
        command, trajectories.getMeanTwoBallLeftPartThreeTrajectory(), IntakeMode.COLORLESS);
    // stops
    stopMoving(command);
    // rotates for the intake to face our fender
    rotateToAngle(command, 115);
    // stops
    stopMoving(command);
    // purges towards the fender
    purgeThroughIntake(command, 5);
    // stops
    stopMoving(command);

    return command;
  }

  // Not competition ready -- and won't be anytime soon
  public Command getMeanShootTwoBallStealTwoBallLeftCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    // resets the gyro
    resetRobotPose(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(command, trajectories.getMeanTwoBallLeftPartOneTrajectory());
    // stops
    stopMoving(command);
    // rotates toward target
    // starts shooting
    autoShot(command);
    // follows second trajectory
    followTrajectoryAndIntakeForced(
        command, trajectories.getMeanTwoBallLeftStealTwoPartTwoTrajectory(), IntakeMode.COLORLESS);
    // stops
    stopMoving(command);
    // follows the third trajectory
    followTrajectoryAndIntakeForced(
        command,
        trajectories.getMeanTwoBallLeftStealTwoPartThreeTrajectory(),
        IntakeMode.COLORLESS);
    // stops
    stopMoving(command);
    // rotates for the shooter to face our hangar
    rotateToAngle(command, 30);
    // stops
    stopMoving(command);
    // purges towards the hangar
    purgeThroughIntake(command, 5);

    return /*new LogCommand("TwoBallLeftStealTwoAuto", */ command;
  }

  public Command getThreeBallRightCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    Trajectory threeBallPartOne = trajectories.getThreeBallRightPartOne(logger);

    // Resets the gyro
    resetRobotPose(command, threeBallPartOne);
    // starts shooting
    blindShotFromInsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_INSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC,
        1);
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(command, threeBallPartOne);
    // stops
    stopMoving(command);
    followTrajectoryAndIntake(command, trajectories.getThreeBallRightPartTwo(logger));
    // stops
    stopMoving(command);
    // rotates to face the target
    rotateToAngle(command, 45);
    // stops
    stopMoving(command);
    // shoots again
    blindShotFromOutsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_OUTSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC,
        1);

    return command;
  }

  public Command getFiveBallRightWaypointsCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();
    Alliance allianceColor = DriverStation.getAlliance();

    resetRobotPose(command, trajectories.getFiveBallRightPartOneWaypoint(logger, allianceColor));
    // starts shooting
    blindShotFromInsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_INSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC,
        1);
    // start pre spinning the shooter and setting the rpm
    preSpinShooter(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_OUTSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC);
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(
        command, trajectories.getFiveBallRightPartOneWaypoint(logger, allianceColor));
    // stops
    stopMoving(command);
    // shoots again
    blindShotFromOutsideTarmac(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_OUTSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC,
        1.5);
    // continues along the trajectory while intaking
    followTrajectoryAndIntake(
        command, trajectories.getFiveBallRightPartTwoWaypoint(logger, allianceColor));
    // stops
    stopMoving(command);
    // pulls intake in
    command.addCommands(new InstantCommand(() -> intake.onStop()));
    // waits for human player to roll ball to bumpers
    wait(command, 1);
    // puts intake out to get human player ball and gets in position to shoot the last two balls
    followTrajectoryAndPreSpinShooter(
        command,
        AutonomousConstants.COMP_SHOOTER_RPM_OUTSIDE_TARMAC,
        AutonomousConstants.COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC,
        trajectories.getFiveBallRightPartThreeWaypoint(logger, allianceColor),
        colorIntakeToggle.getSelected());
    stopMoving(command);
    // shoots one last time
    autoShot(command);
    // stops
    stopMoving(command);

    return command;
  }

  /**
   * Gets an auto path that just drives back and forth, this is made to help characterize our swerve
   * drive.
   */
  public Command getBackAndForthCommand() {
    SequentialCommandGroup command = new SequentialCommandGroup();
    Trajectory partOne = trajectories.getStraightTestPathPartOne(logger);

    resetRobotPose(command, partOne);

    followTrajectory(command, partOne);

    stopMoving(command);

    followTrajectory(command, trajectories.getStraightTestPathPartTwo(logger));

    stopMoving(command);

    return command;
  }

  public Command getCommand() {
    switch (autonomousModeChooser.getSelected()) {
      case ONE_BALL_MID:
        return getOneBallMidCommand();
      case MEAN_SHOOT_TWO_BALL_HANGAR_BALL_LEFT:
        return getMeanShootTwoBallHangarBallLeftCommand();
      case MEAN_SHOOT_TWO_BALL_FENDER_BALL_LEFT:
        return getMeanShootTwoBallFenderBallLeftCommand();
      case MEAN_SHOOT_TWO_BALL_STEAL_TWO_BALL_LEFT:
        return getMeanShootTwoBallStealTwoBallLeftCommand();
      case THREE_BALL_RIGHT:
        return getThreeBallRightCommand();
      case FIVE_BALL_RIGHT_WAYPOINTS:
        return getFiveBallRightWaypointsCommand();
      case BACK_AND_FORTH_TEST:
        return getBackAndForthCommand();
    }

    return getThreeBallRightCommand();
  }

  public Command getIntakeCommand(IntakeMode intakeMode) {
    switch (intakeMode) {
      case COLOR_NORMAL:
        return new ColorIntake(intake, indexerStates, false);
      case COLOR_HARD:
        return new ColorIntake(intake, indexerStates, true);
      case COLORLESS:
        return new ColorlessIntake(intake);
      case PURGE:
        return new PurgeIntake(intake);
    }
    return new ColorIntake(intake, indexerStates, false);
  }

  private void followTrajectory(SequentialCommandGroup command, Trajectory trajectory) {
    command.addCommands(new SwerveDriveToPosition(swerve, pidControllers, () -> trajectory));
  }

  private void followTrajectoryAndIntakeForced(
      SequentialCommandGroup command, Trajectory trajectory, IntakeMode intakeMode) {
    ParallelDeadlineGroup intakeAndDriveCommand =
        new ParallelDeadlineGroup(
            new SwerveDriveToPosition(swerve, pidControllers, () -> trajectory));
    if (intake != null) {
      intakeAndDriveCommand.addCommands(getIntakeCommand(intakeMode));
    }
    intakeAndDriveCommand.addCommands(new DefaultIndexerLoad(indexer));
    command.addCommands(intakeAndDriveCommand);
  }

  private void followTrajectoryAndIntake(SequentialCommandGroup command, Trajectory trajectory) {
    followTrajectoryAndIntakeForced(command, trajectory, colorIntakeToggle.getSelected());
  }

  private void purgeThroughIntake(SequentialCommandGroup command, double timeout) {
    if (shooter != null && indexer != null) {
      command.addCommands(
          new PurgeBack(intake, indexer, shooter)
              .andThen(new LockedSwerveDrive(swerve))
              .withTimeout(timeout));
    }
  }

  // In degrees now
  private void rotateToAngle(SequentialCommandGroup command, double angle) {
    command.addCommands(
        new DoneCycleCommand<>(new RotateToAngleAuto(swerve, gyro, angle), true)
            .withDoneCycles(cmd -> DoneCycleMachine.supplierWithMinCycles(cmd::pidAtSetpoint, 10)),
        new InstantCommand(indexer::spinHold));
  }

  private void blindShotFromOutsideTarmac(
      SequentialCommandGroup command, double shooterRPM, double hoodInput, double timeout) {
    if (shooter != null && indexer != null) {
      command.addCommands(
          new LaunchSetShot(shooter, indexer, smartPCM, shooterRPM, hoodInput)
              .withTimeout(timeout));
    }
  }

  private void autoShot(SequentialCommandGroup command) {
    if (shooter != null && indexer != null) {
      command.addCommands(
          new CalculatedShot(
              shooter,
              indexer,
              smartPCM,
              swerve,
              gyro,
              DriveTrainConstants.SWERVE_ROTATION_SPEED,
              limelight,
              new RotateTowardsPosition(
                  swerve, gyro, pidControllers.thetaPidController, () -> FieldConstants.HUB_CENTER),
              new HeadingRotation(
                  swerve,
                  gyro,
                  DriveTrainConstants.SWERVE_ROTATION_SPEED,
                  FAKE_INPUTS_SUPPLIER,
                  true),
              new DoneCycleCommand<>(
                      new VisionAlignToTarget(
                          swerve,
                          limelight,
                          gyro,
                          DriveTrainConstants.SWERVE_ROTATION_SPEED,
                          () -> true),
                      false)
                  .withDoneCycles(
                      DoneCycleMachine.supplierWithMinCycles(limelight::hasTargets, 8)
                          .withName("Has targets"))
                  .withDoneCycles(
                      cmd ->
                          DoneCycleMachine.supplierWithMinCycles(
                                  cmd::pidAtSetpoint, VisionAlignToTarget.MIN_AT_SETPOINT_CYCLES)
                              .withName("At yaw setpoint"))));
    }
  }

  private void blindShotFromInsideTarmac(
      SequentialCommandGroup command, double shooterRPM, double hoodInput, double timeout) {
    if (shooter != null && indexer != null) {
      command.addCommands(
          new LaunchSetShot(shooter, indexer, smartPCM, shooterRPM, hoodInput)
              .withTimeout(timeout));
    }
  }

  private void followTrajectoryAndPreSpinShooter(
      SequentialCommandGroup command,
      double shooterRPM,
      double hoodInput,
      Trajectory trajectory,
      IntakeMode intakeMode) {
    ParallelDeadlineGroup driveAndPreSpinShooterCommand =
        new ParallelDeadlineGroup(
            new SwerveDriveToPosition(
                swerve,
                pidControllers,
                () -> trajectory,
                () ->
                    VisionCamera.getDesiredAngle(
                        limelight.getBestTarget().get(),
                        gyro.getYaw(),
                        trajectory
                            .sample(trajectory.getTotalTimeSeconds())
                            .poseMeters
                            .getRotation())));
    if (intake != null) {
      driveAndPreSpinShooterCommand.addCommands(getIntakeCommand(intakeMode));
    }
    preSpinShooter(driveAndPreSpinShooterCommand, shooterRPM, hoodInput);
    driveAndPreSpinShooterCommand.addCommands(new DefaultIndexerLoad(indexer));
    command.addCommands(driveAndPreSpinShooterCommand);
  }

  @SuppressWarnings("removal")
  private void preSpinShooter(
      edu.wpi.first.wpilibj2.command.CommandGroupBase command,
      double shooterRPM,
      double hoodInput) {
    SequentialCommandGroup preSpinShooterCommand =
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setRPM(shooterRPM))
                .andThen(new InstantCommand(() -> shooter.setHoodPosition(hoodInput))));
    command.addCommands(preSpinShooterCommand);
  }

  private void wait(SequentialCommandGroup command, double time) {
    command.addCommands(new InstantCommand(() -> swerve.stop()).alongWith(new WaitCommand(time)));
  }

  private void stopMoving(SequentialCommandGroup command) {
    command.addCommands(new InstantCommand(() -> swerve.stop()));
  }

  private void resetRobotPose(SequentialCommandGroup command, Trajectory trajectory) {
    SequentialCommandGroup resetRobotPose = new SequentialCommandGroup();
    resetRobotPose.addCommands(
        new InstantCommand(() -> gyro.setYawAdjustment(trajectory.getInitialPose().getRotation())));
    resetRobotPose.addCommands(
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())));
    command.addCommands(resetRobotPose);
  }

  /*
  private void logMessage(SequentialCommandGroup command, String message) {
    command.addCommands(new InstantCommand(() -> logger.writeToLogFormatted(this, message)));
  }
  */
}
