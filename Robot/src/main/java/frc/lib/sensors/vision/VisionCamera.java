package frc.lib.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.math.PoseEstimationUtil;
import frc.lib.sensors.vision.VisionTargets.VisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;

public interface VisionCamera {
  static class CameraPositionConstants {
    /** Height of the camera off the ground in meters */
    public final double camHeight;
    /** Pitch of the camera, where 0 is level with the ground, and pointed upwards is negative */
    public final Rotation2d camToNormalAngle;
    /**
     * Transform representing position (in meters) and orientation of the camera (relative to the
     * robot)
     */
    public final Transform3d robotCenterToCamera;

    /**
     * @param robotCenterToCamera Transform representing position (in meters) and orientation of the
     *     camera (relative to the robot)
     */
    public CameraPositionConstants(Transform3d robotCenterToCamera) {
      camHeight = robotCenterToCamera.getZ();
      camToNormalAngle = new Rotation2d(robotCenterToCamera.getRotation().getY());
      this.robotCenterToCamera = robotCenterToCamera;
    }
  }

  boolean hasTargets();

  void takeSnapshot();

  int getRunningPipeline();

  void setPipeline(int index);

  void setLed(VisionLEDMode mode);

  CameraPositionConstants getCameraPositionConsts();

  /**
   * Estimates the translational distance (not including height) from the camera to the target
   * (tape, tag, etc.). Works better the greater the height differential is between the target and
   * the robot.
   *
   * @param target Vision target to estimate distance to
   * @param targetHeight Height of the target in meters
   * @return Translational distance to target in meters
   */
  default double getEstimatedDistance(VisionTarget target, double targetHeight) {
    return PoseEstimationUtil.calculateDistanceToTarget(
        getCameraPositionConsts().camHeight,
        targetHeight,
        getCameraPositionConsts().camToNormalAngle,
        target.getY(),
        target.getX());
  }

  /**
   * Calculates an estimated robot position based on a given translational distance and angle from
   * the camera to a target whose position is provided, and the angle to the target. Only call this
   * method if you know your translational distance is correct.
   *
   * @param target Reference vision target obtained from this camera
   * @param targetPos Absolute translational position of the vision target
   * @param translationalDistance Translational distance from the target in meters
   * @param currentAngle Current yaw of the robot
   */
  default Pose2d getRobotPoseFromDistance(
      VisionTarget target,
      Translation2d targetPos,
      double translationalDistance,
      Rotation2d currentAngle) {
    Transform3d robotCenterToCamera = getCameraPositionConsts().robotCenterToCamera;
    Rotation2d relativeCameraRotation = robotCenterToCamera.getRotation().toRotation2d();
    return PoseEstimationUtil.getRobotPoseFrom2d(
        targetPos,
        translationalDistance,
        currentAngle,
        currentAngle.plus(relativeCameraRotation).plus(target.getX()),
        new Transform2d(
            robotCenterToCamera.getTranslation().toTranslation2d(), relativeCameraRotation));
  }

  /**
   * @param target Vision target to turn to
   * @param currentAngle The current angle of the robot
   * @param defaultAngle The rotation to turn to if {@code target} is null
   * @return If the camera has a target, the angle needed to face the center of that target,
   *     otherwise, this returns a default angle.
   */
  static Rotation2d getDesiredAngle(
      VisionTarget target, Rotation2d currentAngle, Rotation2d defaultAngle) {
    return (target != null) ? currentAngle.plus(target.getX()) : defaultAngle;
  }
}
