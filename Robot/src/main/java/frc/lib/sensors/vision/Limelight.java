package frc.lib.sensors.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.math.EpsilonUtil;
import frc.lib.misc.Util;
import frc.lib.sensors.vision.LimelightHelpers.CoordinateSpace;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightResults;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightTarget_Generic;
import frc.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc.lib.sensors.vision.VisionCameras.ReflectiveTapeCamera;
import frc.lib.sensors.vision.VisionTargets.AprilTagLimelightTarget;
import frc.lib.sensors.vision.VisionTargets.LimelightTarget;
import frc.lib.sensors.vision.VisionTargets.RetroreflectiveLimelightTarget;
import frc.lib.sensors.vision.VisionTargets.VisionTarget;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight extends CameraSubsystem implements ReflectiveTapeCamera, AprilTagCamera {
  static class TargetEntries {
    final DoubleSubscriber xSub, ySub, areaSub, skewSub, ambiguitySub;
    final IntegerSubscriber idSub;
    final DoubleArraySubscriber camtranSub;

    TargetEntries(NetworkTable table) {
      xSub = table.getDoubleTopic("tx").subscribe(0.0);
      ySub = table.getDoubleTopic("ty").subscribe(0.0);
      areaSub = table.getDoubleTopic("ta").subscribe(0.0);
      skewSub = table.getDoubleTopic("ts").subscribe(0.0);
      idSub = table.getIntegerTopic("tid").subscribe(-1);
      camtranSub =
          table
              .getDoubleArrayTopic("targetpose_cameraspace")
              .subscribe(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      // Not a real ambiguity topic
      ambiguitySub = table.getDoubleTopic("tpa").subscribe(0);
    }
  }

  private static final Map<VisionLEDMode, Integer> ledModeToEntryValue =
      Collections.unmodifiableMap(
          Map.of(
              VisionLEDMode.kDefault,
              0,
              VisionLEDMode.kOn,
              3,
              VisionLEDMode.kOff,
              1,
              VisionLEDMode.kBlink,
              2));

  private static final int DEFAULT_PIPE = 0;
  private int snapshotCycles = 1;

  private RetroreflectiveLimelightTarget bestReflectiveTarget;
  private AprilTagLimelightTarget bestAprilTagTarget;
  private LimelightJSONTargets.LimelightResults curJsonResult;

  private final TargetEntries targetEntries;
  private Supplier<double[]> botposeSupplier;
  private final DoubleSubscriber hasTargetSub, currentPipeSub;
  private final DoublePublisher ledControlPub, pipeControlPub, snapshotPub;

  private Optional<DigitalOutput> extraLedDio;

  public Limelight(String name, CameraPositionConstants cameraConsts, LogManager logger) {
    super(name, cameraConsts, logger);

    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    targetEntries = new TargetEntries(table);
    hasTargetSub = table.getDoubleTopic("tv").subscribe(0.0);
    currentPipeSub = table.getDoubleTopic("getpipe").subscribe(0.0);
    ledControlPub = table.getDoubleTopic("ledMode").publish();
    pipeControlPub = table.getDoubleTopic("pipeline").publish();
    snapshotPub = table.getDoubleTopic("snapshot").publish();

    extraLedDio = Optional.empty();
    setLed(VisionLEDMode.kOff);
    setPipeline(DEFAULT_PIPE);
  }

  public Limelight(
      String name, CameraPositionConstants cameraConsts, int secondLedDio, LogManager logger) {
    this(name, cameraConsts, logger);
    extraLedDio = Optional.of(new DigitalOutput(secondLedDio));
  }

  @Override
  public boolean hasTargets() {
    return EpsilonUtil.epsilonEquals(hasTargetSub.get(), 1.0);
  }

  @Override
  public int getRunningPipeline() {
    return (int) currentPipeSub.get();
  }

  @Override
  public Optional<? extends LimelightTarget> getBestTarget() {
    // If the generic method is called, the subclass doesn't matter
    return getBestRetroreflectiveTarget();
  }

  @Override
  public List<? extends LimelightTarget> getTargets() {
    List<LimelightTarget> targetList = new ArrayList<>();
    targetList.addAll(getRetroreflectiveTargets());
    targetList.addAll(getAprilTagTargets());
    return targetList;
  }

  private <T extends VisionTarget, R extends LimelightTarget_Generic> List<T> getJSONTargets(
      Function<R, T> targetFromJson, Function<LimelightJSONTargets.Results, R[]> targetResults) {
    LimelightJSONTargets.Results jsonResults = getResults().targetingResults;
    List<T> targets = new ArrayList<>();
    for (R target : targetResults.apply(jsonResults)) {
      T constructedTarget = targetFromJson.apply(target);
      targets.add(constructedTarget);
    }
    return targets;
  }

  @Override
  public Optional<RetroreflectiveLimelightTarget> getBestRetroreflectiveTarget() {
    // Constructs a "retroreflective" target from the best target information.
    // There's no guarantee what kind of target it is. (Same goes for AprilTag targets)
    if (bestReflectiveTarget == null && hasTargets()) {
      bestReflectiveTarget = new RetroreflectiveLimelightTarget(targetEntries);
    }
    return Optional.ofNullable(bestReflectiveTarget);
  }

  @Override
  public List<RetroreflectiveLimelightTarget> getRetroreflectiveTargets() {
    return getJSONTargets(RetroreflectiveLimelightTarget::new, results -> results.targets_Retro);
  }

  @Override
  public Optional<AprilTagLimelightTarget> getBestAprilTagTarget() {
    if (bestAprilTagTarget == null && hasTargets()) {
      bestAprilTagTarget = new AprilTagLimelightTarget(targetEntries);
    }
    return Optional.ofNullable(bestAprilTagTarget);
  }

  @Override
  public List<AprilTagLimelightTarget> getAprilTagTargets() {
    return getJSONTargets(AprilTagLimelightTarget::new, results -> results.targets_Fiducials);
  }

  public Pose3d getRobotPose() {
    double[] botpose = botposeSupplier.get();
    return LimelightHelpers.toGeometry3D(botpose, CoordinateSpace.Field, Pose3d::new);
  }

  public LimelightResults getResults() {
    return Util.lazyInitialize(
        results -> curJsonResult = results,
        () -> LimelightHelpers.getLatestResults(name),
        curJsonResult);
  }

  @Override
  public void setLed(VisionLEDMode controlMode) {
    ledControlPub.set(ledModeToEntryValue.get(controlMode));
    extraLedDio.ifPresent(out -> out.set(controlMode == VisionLEDMode.kOn));
  }

  @Override
  public void setPipeline(int pipeline) {
    pipeControlPub.set(pipeline);
  }

  @Override
  public void takeSnapshot() {
    setSnapshotState(0);
    setSnapshotState(1);
    snapshotCycles = 10;
  }

  private void setSnapshotState(int state) {
    snapshotPub.set(state);
  }

  @Override
  public void periodic() {
    bestReflectiveTarget = null;
    bestAprilTagTarget = null;
    curJsonResult = null;

    snapshotCycles--;
    if (snapshotCycles == 0) {
      setSnapshotState(0);
    }
    snapshotCycles = Math.max(snapshotCycles, -1);
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    putDashboardPrints();
    LeveledSmartDashboard.INFO.putNumber(
        name + "-X", getBestTarget().map(target -> target.getX().getDegrees()).orElse(0.));
    LeveledSmartDashboard.INFO.putNumber(
        name + "-Y", getBestTarget().map(target -> target.getY().getDegrees()).orElse(0.));
    LeveledSmartDashboard.HIGH.putBoolean(name + "-HasTarget", hasTargets());
    LeveledSmartDashboard.INFO.putNumber(
        name + "-Area", getBestTarget().map(target -> target.getArea()).orElse(0.));
    LeveledSmartDashboard.INFO.putNumber(name + "-Pipe", getRunningPipeline());
  }

  @Override
  public void onStart() {
    botposeSupplier =
        DriverStation.getAlliance() == Alliance.Blue
            ? () -> LimelightHelpers.getBotPose_wpiBlue(name)
            : () -> LimelightHelpers.getBotPose_wpiRed(name);

    setLed(VisionLEDMode.kOn);
  }

  @Override
  public void onStop() {
    setLed(VisionLEDMode.kOff);
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    super.setUpTrackables(logger);
    logger
        .getLogFile(name + "/TargetXY")
        .setDefaultFrequency(10)
        .addTracker(
            "_X", () -> getBestTarget().map(target -> target.getX().getDegrees()).orElse(0.))
        .addTracker(
            "_Y", () -> getBestTarget().map(target -> target.getY().getDegrees()).orElse(0.));
  }
}
