package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import java.util.Collection;
import java.util.Optional;

public final class PoseEstimationUtil {
  private PoseEstimationUtil() {}

  /**
   * Estimates camera range to a target using the target's elevation, and only 2D vision
   * information. This method can produce more stable results than SolvePnP when well tuned, if the
   * full 6D robot pose is not required. Note that this method requires the camera to have 0 roll
   * (not be skewed clockwise or CCW relative to the floor), and for there to exist a height
   * differential between goal and camera. The larger this differential, the more accurate the
   * distance estimate will be.
   */
  public static double calculateDistanceToTarget(
      double cameraHeightMeters,
      double targetHeightMeters,
      Rotation2d cameraPitch,
      Rotation2d targetPitch,
      Rotation2d targetYaw) {
    return (targetHeightMeters - cameraHeightMeters)
        / (cameraPitch.plus(targetPitch).getTan() * targetYaw.getCos());
  }

  /**
   * This method will calculate an estimated robot position based on a given translational distance
   * and angle from camera to a target whose position is provided, and the angle to the target. Only
   * call this method if you know your translational distance is correct.
   *
   * @param translationalDistance Translational distance from the target in meters
   * @see {@link #calculateDistanceToTarget(double, double, Rotation2d, Rotation2d, Rotation2d)}
   */
  public static Pose2d getRobotPoseFrom2d(
      Translation2d targetPos,
      double translationalDistance,
      Rotation2d currentAngle,
      Rotation2d angleToTarget,
      Transform2d robotCenterToCamera) {
    // The robot's quadrant on the field does not matter when doing the following calculations.
    Translation2d relativeTransformToCamera =
        new Translation2d(
            -angleToTarget.getCos() * translationalDistance,
            -angleToTarget.getSin() * translationalDistance);
    Translation2d estimatedCameraPos = targetPos.plus(relativeTransformToCamera);
    // Transform pose from camera to robot center
    return new Pose2d(estimatedCameraPos, currentAngle).transformBy(robotCenterToCamera.inverse());
  }

  /**
   * Returns the pose of the robot center from tranform information between the camera and a
   * reference point (such as a vision target).
   */
  public static Pose2d getRobotPoseFromTransform(
      Pose3d referencePose, Transform3d cameraToReference, Transform3d robotCenterToCamera) {
    return referencePose
        .transformBy(cameraToReference.inverse())
        .transformBy(robotCenterToCamera.inverse())
        .toPose2d();
  }

  /**
   * Returns the pose of a reference point (such as a vision target) from tranform information
   * between the camera and the reference point.
   */
  public static Pose3d getTargetPoseFromTransform(
      Pose2d robotPose, Transform3d cameraToTarget, Transform3d robotCenterToCamera) {
    return new Pose3d(robotPose).transformBy(robotCenterToCamera).transformBy(cameraToTarget);
  }

  public static Optional<? extends AprilTagTarget> getBestTarget(
      AprilTagCamera camera, Collection<Integer> ids) {
    Optional<? extends AprilTagTarget> bestTarget = camera.getBestAprilTagTarget();
    if (bestTarget.isEmpty() || ids.contains(bestTarget.get().getId())) {
      return bestTarget;
    }
    return camera.getAprilTagTargets().stream()
        .filter(target -> ids.contains(target.getId()))
        .sorted(
            (a, b) ->
                a.getCameraToTarget().getTranslation().getNorm()
                        < b.getCameraToTarget().getTranslation().getNorm()
                    ? -1
                    : 1)
        .findFirst();
  }
}
