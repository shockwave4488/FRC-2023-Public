package frc.lib.sensors.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.math.PoseEstimationUtil;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc.lib.sensors.vision.VisionTargets.Pose3dTarget;
import frc.lib.sensors.vision.VisionTargets.RetroreflectiveTarget;
import frc.lib.sensors.vision.VisionTargets.VisionTarget;
import java.util.List;
import java.util.Optional;

public class VisionCameras {
  private VisionCameras() {}

  public interface TargetCamera extends VisionCamera {
    Optional<? extends VisionTarget> getBestTarget();

    List<? extends VisionTarget> getTargets();
  }

  public interface ReflectiveTapeCamera extends TargetCamera {
    Optional<? extends RetroreflectiveTarget> getBestRetroreflectiveTarget();

    List<? extends RetroreflectiveTarget> getRetroreflectiveTargets();
  }

  public interface Pose3dTargetCamera extends TargetCamera {
    Optional<? extends Pose3dTarget> getBestPose3dTarget();

    List<? extends Pose3dTarget> getPose3dTargets();

    default void putDashboardPrints() {
      if (getBestPose3dTarget().isPresent()) {
        LeveledSmartDashboard.INFO.putString(
            "Best target transform", getBestPose3dTarget().get().getCameraToTarget().toString());
      } else {
        LeveledSmartDashboard.INFO.putString("Best target transform", "No target");
      }
    }

    default Pose2d getRobotPoseFromBestTransform(Pose3d targetPose) {
      return PoseEstimationUtil.getRobotPoseFromTransform(
          targetPose,
          getBestPose3dTarget().get().getCameraToTarget(),
          getCameraPositionConsts().robotCenterToCamera);
    }
  }

  public interface AprilTagCamera extends Pose3dTargetCamera {
    @Override
    default Optional<? extends Pose3dTarget> getBestPose3dTarget() {
      return getBestAprilTagTarget();
    }

    @Override
    default List<? extends Pose3dTarget> getPose3dTargets() {
      return getAprilTagTargets();
    }

    Optional<? extends AprilTagTarget> getBestAprilTagTarget();

    List<? extends AprilTagTarget> getAprilTagTargets();

    @Override
    default void putDashboardPrints() {
      if (getBestAprilTagTarget().isPresent()) {
        LeveledSmartDashboard.HIGH.putNumber(
            "Best target ID", getBestAprilTagTarget().get().getId());
      } else {
        LeveledSmartDashboard.HIGH.putNumber("Best target ID", -1);
      }
      /*
      LeveledNetTable.INFO.putNumberArray(
          "Target IDs",
          getAprilTagTargets().stream().mapToDouble(target -> target.getId()).toArray());
      */
    }

    default AprilTag[] getVisibleAprilTags(Pose2d robotPose) {
      return getAprilTagTargets().stream()
          .map(
              tagTarget ->
                  tagTarget.toAprilTag(robotPose, getCameraPositionConsts().robotCenterToCamera))
          .toArray(AprilTag[]::new);
    }
  }
}
