package frc.lib.sensors.vision;

import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc.lib.sensors.vision.VisionCameras.ReflectiveTapeCamera;
import frc.lib.sensors.vision.VisionTargets.AprilTagPhotonTarget;
import frc.lib.sensors.vision.VisionTargets.PhotonTarget;
import frc.lib.sensors.vision.VisionTargets.RetroreflectivePhotonTarget;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraSubsystem extends CameraSubsystem
    implements ReflectiveTapeCamera, AprilTagCamera {
  protected final PhotonCamera camera;
  protected PhotonPipelineResult curResult;
  private RetroreflectivePhotonTarget bestReflectiveTarget;
  private AprilTagPhotonTarget bestAprilTagTarget;

  public PhotonCameraSubsystem(
      String name, CameraPositionConstants cameraConsts, LogManager logger) {
    super(name, cameraConsts, logger);
    camera = new PhotonCamera(name);
  }

  @Override
  public boolean hasTargets() {
    return getCurResult().hasTargets();
  }

  @Override
  public Optional<? extends PhotonTarget> getBestTarget() {
    return getBestRetroreflectiveTarget();
  }

  @Override
  public List<? extends PhotonTarget> getTargets() {
    return curResult.getTargets().stream().map(target -> new PhotonTarget(target)).toList();
  }

  @Override
  public Optional<RetroreflectivePhotonTarget> getBestRetroreflectiveTarget() {
    if (bestReflectiveTarget == null && hasTargets()) {
      bestReflectiveTarget = new RetroreflectivePhotonTarget(curResult.getBestTarget());
    }
    return Optional.ofNullable(bestReflectiveTarget);
  }

  @Override
  public List<RetroreflectivePhotonTarget> getRetroreflectiveTargets() {
    return curResult.getTargets().stream()
        .map(target -> new RetroreflectivePhotonTarget(target))
        .toList();
  }

  private Stream<PhotonTrackedTarget> getAprilTagPhotonTargets() {
    return curResult.getTargets().stream().filter(target -> target.getFiducialId() != -1);
  }

  @Override
  public Optional<AprilTagPhotonTarget> getBestAprilTagTarget() {
    if (bestAprilTagTarget == null && hasTargets()) {
      Optional<AprilTagPhotonTarget> optionalTarget =
          getAprilTagPhotonTargets().findFirst().map(target -> new AprilTagPhotonTarget(target));
      bestAprilTagTarget = optionalTarget.orElse(null);
    }
    return Optional.ofNullable(bestAprilTagTarget);
  }

  @Override
  public List<AprilTagPhotonTarget> getAprilTagTargets() {
    return getAprilTagPhotonTargets().map(target -> new AprilTagPhotonTarget(target)).toList();
  }

  @Override
  public int getRunningPipeline() {
    return camera.getPipelineIndex();
  }

  @Override
  public void setPipeline(int index) {
    camera.setPipelineIndex(index);
  }

  @Override
  public void setLed(VisionLEDMode mode) {
    camera.setLED(mode);
  }

  @Override
  public void takeSnapshot() {
    camera.takeOutputSnapshot();
  }

  private PhotonPipelineResult getCurResult() {
    if (curResult == null) {
      curResult = camera.getLatestResult();
    }
    return curResult;
  }

  @Override
  public void periodic() {
    curResult = camera.getLatestResult();
    bestReflectiveTarget = null;
    bestAprilTagTarget = null;
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    putDashboardPrints();
    LeveledSmartDashboard.HIGH.putBoolean("Camera has targets", getCurResult().hasTargets());
    LeveledSmartDashboard.INFO.putNumber(
        "Latest result timestamp", getCurResult().getTimestampSeconds());
  }
}
