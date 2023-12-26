package frc.robot.autonomous.modes.eruption;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.ArrayList;
import java.util.List;

public class AutonomousTrajectories {

  private final LogManager logger;
  Trajectory trajectory = new Trajectory();
  Trajectory oneBallMidTrajectory = new Trajectory();
  Trajectory meanTwoBallLeftPartOneTrajectory = new Trajectory();
  Trajectory meanTwoBallLeftPartTwoTrajectory = new Trajectory();
  Trajectory meanTwoBallLeftPartThreeTrajectory = new Trajectory();
  Trajectory meanTwoBallLeftStealTwoPartTwoTrajectory = new Trajectory();
  Trajectory meanTwoBallLeftStealTwoPartThreeTrajectory = new Trajectory();
  Trajectory threeBallRightPartOneTrajectory = new Trajectory();
  Trajectory threeBallRightPartTwoTrajectory = new Trajectory();
  Trajectory fiveBallRightPartOneRedWaypointTrajectory = new Trajectory();
  Trajectory fiveBallRightPartTwoRedWaypointTrajectory = new Trajectory();
  Trajectory fiveBallRightPartThreeRedWaypointTrajectory = new Trajectory();
  Trajectory fiveBallRightPartOneBlueWaypointTrajectory = new Trajectory();
  Trajectory fiveBallRightPartTwoBlueWaypointTrajectory = new Trajectory();
  Trajectory fiveBallRightPartThreeBlueWaypointTrajectory = new Trajectory();
  Trajectory straightPathTestPartOne = new Trajectory();
  Trajectory straightPathTestPartTwo = new Trajectory();
  TrajectoryConfig config;
  TrajectoryConfig reverseConfig;

  public AutonomousTrajectories(SwerveDrive swerve, LogManager logger) {
    this.logger = logger;
    config =
        new TrajectoryConfig(
                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL)
            .setKinematics(swerve.getKinematics()); // max speed, max acceleration
    reverseConfig =
        new TrajectoryConfig(
                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL)
            .setKinematics(swerve.getKinematics()); // max speed, max acceleration
    reverseConfig.setReversed(true);
    // mean 2 ball auto paths
    meanTwoBallLeftPartOneTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.8, 6.0, new Rotation2d(-Math.PI / 4)),
            new ArrayList<Translation2d>(),
            new Pose2d(5, 6.2, new Rotation2d(-Math.PI / 8)),
            reverseConfig);
    meanTwoBallLeftPartTwoTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 6.2, new Rotation2d(-Math.PI / 8)),
            new ArrayList<Translation2d>(),
            new Pose2d(6.6, 6.2, new Rotation2d(-Math.PI / 4)),
            config);
    meanTwoBallLeftPartThreeTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.6, 6.2, new Rotation2d(-Math.PI / 4)),
            new ArrayList<Translation2d>(),
            new Pose2d(6.3, 7.3, new Rotation2d(-Math.PI / 4)),
            config);
    // Meaner left auto
    meanTwoBallLeftStealTwoPartTwoTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 6.2, new Rotation2d(-Math.PI / 4)),
            new ArrayList<Translation2d>(),
            new Pose2d(4.48, 3.2, new Rotation2d(Math.PI / 2)),
            reverseConfig);
    meanTwoBallLeftStealTwoPartThreeTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.48, 3.2, new Rotation2d(Math.PI / 2)),
            new ArrayList<Translation2d>(),
            new Pose2d(6.1, 7.3, new Rotation2d(-3 * Math.PI / 4)),
            config);
    // normal 1 ball auto path
    oneBallMidTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.0, 4.0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(4.0, 4.01, new Rotation2d(0)),
            reverseConfig);
    // normal three ball auto paths
    threeBallRightPartOneTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(8.27, 1.79, new Rotation2d(Math.PI / 2)),
            List.of(new Translation2d(8.27, 1.60)),
            new Pose2d(7.64, 0.78, new Rotation2d(1.39)),
            reverseConfig);
    threeBallRightPartTwoTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(7.64, 0.78, new Rotation2d(1.39)),
            new ArrayList<Translation2d>(),
            new Pose2d(5.17, 1.86, new Rotation2d(0.0)),
            reverseConfig);
    // waypoint five ball auto paths Red
    fiveBallRightPartOneRedWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(8.27, 1.79, new Rotation2d(Math.PI / 2)),
            List.of(
                new Translation2d(8.6, 0.9),
                new Translation2d(7.64, 0.78),
                new Translation2d(6.0, 2.0)),
            new Pose2d(5.12, 2.41, new Rotation2d(0.55)),
            reverseConfig); // needs to be reversed
    fiveBallRightPartTwoRedWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.12, 2.41, new Rotation2d(0.55)),
            new ArrayList<Translation2d>(),
            new Pose2d(1.35, 1.65, new Rotation2d(Math.PI / 4)),
            reverseConfig); // needs to be reversed
    // (1.25, 1.55) -> (1.35, 1.65)
    fiveBallRightPartThreeRedWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.35, 1.65, new Rotation2d(Math.PI / 4)),
            new ArrayList<Translation2d>(),
            new Pose2d(5.17, 2.36, new Rotation2d(0.55)),
            config);
    // waypoint five ball auto paths Blue
    fiveBallRightPartOneBlueWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(8.27, 1.79, new Rotation2d(Math.PI / 2)),
            List.of(
                new Translation2d(8.6, 0.9),
                new Translation2d(7.64, 0.78),
                new Translation2d(6.0, 2.0)),
            new Pose2d(5.12, 2.41, new Rotation2d(0.55)),
            reverseConfig); // needs to be reversed
    fiveBallRightPartTwoBlueWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.12, 2.41, new Rotation2d(0.55)),
            new ArrayList<Translation2d>(),
            new Pose2d(1.25, 1.9, new Rotation2d(Math.PI / 4)),
            reverseConfig); // needs to be reversed
    fiveBallRightPartThreeBlueWaypointTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.25, 1.9, new Rotation2d(Math.PI / 4)),
            new ArrayList<Translation2d>(),
            new Pose2d(5.17, 2.36, new Rotation2d(0.55)),
            config);
    // test path parts
    straightPathTestPartOne =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(5, 0, new Rotation2d(0)),
            config);
    straightPathTestPartTwo =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(0, 0, new Rotation2d(0)),
            reverseConfig);
  }

  public Trajectory getOneBallMid() {
    // logStates(twoBallLeftTrajectory);
    return oneBallMidTrajectory;
  }

  public Trajectory getMeanTwoBallLeftPartOneTrajectory() {
    // logStates(meanTwoBallLeftPartOneTrajectory);
    return meanTwoBallLeftPartOneTrajectory;
  }

  public Trajectory getMeanTwoBallLeftPartTwoTrajectory() {
    // logStates(meanTwoBallLeftPartTwoTrajectory);
    return meanTwoBallLeftPartTwoTrajectory;
  }

  public Trajectory getMeanTwoBallLeftPartThreeTrajectory() {
    // logStates(meanTwoBallLeftPartThreeTrajectory);
    return meanTwoBallLeftPartThreeTrajectory;
  }

  public Trajectory getMeanTwoBallLeftStealTwoPartTwoTrajectory() {
    // logStates(meanTwoBallLeftStealTwoPartOneTrajectory);
    return meanTwoBallLeftStealTwoPartTwoTrajectory;
  }

  public Trajectory getMeanTwoBallLeftStealTwoPartThreeTrajectory() {
    // logStates(meanTwoBallLeftStealTwoPartTwoTrajectory);
    return meanTwoBallLeftStealTwoPartThreeTrajectory;
  }

  public Trajectory getThreeBallRightPartOne(LogManager logger) {
    // logStates(threeBallRightPartOneTrajectory);
    return threeBallRightPartOneTrajectory;
  }

  public Trajectory getThreeBallRightPartTwo(LogManager logger) {
    // logStates(threeBallRightPartTwoTrajectory);
    return threeBallRightPartTwoTrajectory;
  }

  public Trajectory getFiveBallRightPartOneWaypoint(LogManager logger, Alliance allianceColor) {
    if (allianceColor == Alliance.Red) {
      // logStates(fiveBallRightPartOneRedWaypointTrajectory);
      trajectory = fiveBallRightPartOneRedWaypointTrajectory;
    } else {
      // logStates(fiveBallRightPartOneBlueWaypointTrajectory);
      trajectory = fiveBallRightPartOneBlueWaypointTrajectory;
    }
    return trajectory;
  }

  public Trajectory getFiveBallRightPartTwoWaypoint(LogManager logger, Alliance allianceColor) {
    if (allianceColor == Alliance.Red) {
      // logStates(fiveBallRightPartTwoRedWaypointTrajectory);
      trajectory = fiveBallRightPartTwoRedWaypointTrajectory;
    } else {
      // logStates(fiveBallRightPartTwoBlueWaypointTrajectory);
      trajectory = fiveBallRightPartTwoBlueWaypointTrajectory;
    }
    return trajectory;
  }

  public Trajectory getFiveBallRightPartThreeWaypoint(LogManager logger, Alliance allianceColor) {
    if (allianceColor == Alliance.Red) {
      // logStates(fiveBallRightPartThreeRedWaypointTrajectory);
      trajectory = fiveBallRightPartThreeRedWaypointTrajectory;
    } else {
      // logStates(fiveBallRightPartThreeBlueWaypointTrajectory);
      trajectory = fiveBallRightPartThreeBlueWaypointTrajectory;
    }
    return trajectory;
  }

  public Trajectory getStraightTestPathPartOne(LogManager logger) {
    logStates(straightPathTestPartOne);
    return straightPathTestPartOne;
  }

  public Trajectory getStraightTestPathPartTwo(LogManager logger) {
    logStates(straightPathTestPartTwo);
    return straightPathTestPartTwo;
  }

  private void logStates(Trajectory trajectory) {
    logger.getMainLog().println(LogLevel.INFO, trajectory.toString() + " starts here");
    for (Trajectory.State state : trajectory.getStates()) {
      logger.getMainLog().println(LogLevel.INFO, state.toString());
    }
    logger.getMainLog().println(LogLevel.INFO, trajectory.toString() + " ends here");
  }
}
