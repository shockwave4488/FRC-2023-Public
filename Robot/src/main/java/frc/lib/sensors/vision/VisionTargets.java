package frc.lib.sensors.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.math.PoseEstimationUtil;
import frc.lib.misc.Util;
import frc.lib.sensors.vision.LimelightHelpers.CoordinateSpace;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightTarget_Fiducial;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightTarget_Generic;
import frc.lib.sensors.vision.LimelightJSONTargets.LimelightTarget_Retro;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionTargets {
  private VisionTargets() {}

  public enum TargetType {
    Retroreflective,
    Fudicial,
    NeuralDetector
  }

  public interface VisionTarget {
    /**
     * @return The horizontal angle difference between the center of the limelight's view and where
     *     its detected target is on its view. If the target is to the left of the center, the angle
     *     is positive.
     */
    Rotation2d getX();

    /**
     * @return The vertical angle difference between the center of the limelight's view and where
     *     its detected target is on its view. If the target is above the center, the angle is
     *     negative.
     */
    Rotation2d getY();

    /**
     * @return The percentage (0-100) of the image the target takes up.
     */
    double getArea();

    /**
     * @return The rotation of the bounding box of the target, in the range 0 to -90 degrees
     */
    Rotation2d getSkew();
  }

  public interface RetroreflectiveTarget extends VisionTarget {}

  public interface Pose3dTarget extends VisionTarget {
    Transform3d getCameraToTarget();

    double getPoseAmbiguity();
  }

  public interface AprilTagTarget extends Pose3dTarget {
    int getId();

    default AprilTag toAprilTag(Pose2d robotPose, Transform3d robotCenterToCamera) {
      return new AprilTag(
          getId(),
          PoseEstimationUtil.getTargetPoseFromTransform(
              robotPose, getCameraToTarget(), robotCenterToCamera));
    }
  }

  public static class LimelightTarget implements VisionTarget {
    private Rotation2d xOffset, yOffset, skew;
    private final Supplier<Rotation2d> xOffsetSupplier, yOffsetSupplier, skewSupplier;
    private Double area;
    private final Supplier<Double> areaSupplier;

    @Override
    public Rotation2d getX() {
      return Util.lazyInitialize(offset -> xOffset = offset, xOffsetSupplier, xOffset);
    }

    @Override
    public Rotation2d getY() {
      return Util.lazyInitialize(offset -> yOffset = offset, yOffsetSupplier, yOffset);
    }

    @Override
    public double getArea() {
      return Util.lazyInitialize(targetArea -> area = targetArea, areaSupplier, area);
    }

    @Override
    public Rotation2d getSkew() {
      return Util.lazyInitialize(targetSkew -> skew = targetSkew, skewSupplier, skew);
    }

    LimelightTarget(Limelight.TargetEntries targetTable) {
      xOffsetSupplier = () -> Rotation2d.fromDegrees(-targetTable.xSub.get());
      yOffsetSupplier = () -> Rotation2d.fromDegrees(-targetTable.ySub.get());
      skewSupplier = () -> Rotation2d.fromDegrees(-targetTable.skewSub.get());
      areaSupplier = targetTable.areaSub::get;
    }

    LimelightTarget(LimelightTarget_Generic result) {
      xOffsetSupplier = () -> Rotation2d.fromDegrees(-result.tx);
      yOffsetSupplier = () -> Rotation2d.fromDegrees(-result.ty);
      skewSupplier = () -> Rotation2d.fromDegrees(-result.ts);
      areaSupplier = () -> result.ta;
    }
  }

  public static class RetroreflectiveLimelightTarget extends LimelightTarget
      implements RetroreflectiveTarget {
    RetroreflectiveLimelightTarget(Limelight.TargetEntries targetTable) {
      super(targetTable);
    }

    RetroreflectiveLimelightTarget(LimelightTarget_Retro result) {
      super(result);
    }
  }

  public static class AprilTagLimelightTarget extends LimelightTarget implements AprilTagTarget {
    private Integer tagId = null;
    private final Supplier<Integer> tagIdSupplier;
    private Transform3d cameraToTarget;
    private final Supplier<Transform3d> transformSupplier;
    private Double poseAmbiguity;
    private final Supplier<Double> ambiguitySupplier;

    @Override
    public int getId() {
      return Util.lazyInitialize(id -> tagId = id, tagIdSupplier, tagId);
    }

    @Override
    public Transform3d getCameraToTarget() {
      return Util.lazyInitialize(
          transform -> cameraToTarget = transform, transformSupplier, cameraToTarget);
    }

    @Override
    public double getPoseAmbiguity() {
      return Util.lazyInitialize(
          ambiguity -> poseAmbiguity = ambiguity, ambiguitySupplier, poseAmbiguity);
    }

    AprilTagLimelightTarget(Limelight.TargetEntries targetTable) {
      super(targetTable);
      tagIdSupplier = () -> (int) targetTable.idSub.get();
      double[] camtran = targetTable.camtranSub.get();
      transformSupplier =
          () -> LimelightHelpers.toGeometry3D(camtran, CoordinateSpace.Camera, Transform3d::new);
      ambiguitySupplier = targetTable.ambiguitySub::get;
    }

    AprilTagLimelightTarget(LimelightTarget_Fiducial result) {
      super(result);
      tagIdSupplier = () -> (int) result.fiducialID;
      double[] camtran = result.targetPose_CameraSpace;
      transformSupplier =
          () -> LimelightHelpers.toGeometry3D(camtran, CoordinateSpace.Camera, Transform3d::new);
      ambiguitySupplier = () -> 0.;
    }
  }

  public static class PhotonTarget implements VisionTarget {
    protected final PhotonTrackedTarget target;
    private final Rotation2d xOffset, yOffset, skew;

    @Override
    public Rotation2d getX() {
      return xOffset;
    }

    @Override
    public Rotation2d getY() {
      return yOffset;
    }

    @Override
    public double getArea() {
      return target.getArea();
    }

    @Override
    public Rotation2d getSkew() {
      return skew;
    }

    public List<TargetCorner> getCorners() {
      return target.getDetectedCorners();
    }

    PhotonTarget(PhotonTrackedTarget target) {
      this.target = target;
      xOffset = Rotation2d.fromDegrees(-target.getYaw());
      yOffset = Rotation2d.fromDegrees(-target.getPitch());
      skew = Rotation2d.fromDegrees(-target.getSkew());
    }
  }

  public static class RetroreflectivePhotonTarget extends PhotonTarget
      implements RetroreflectiveTarget {
    RetroreflectivePhotonTarget(PhotonTrackedTarget target) {
      super(target);
    }
  }

  public static class AprilTagPhotonTarget extends PhotonTarget implements AprilTagTarget {
    @Override
    public int getId() {
      return target.getFiducialId();
    }

    @Override
    public Transform3d getCameraToTarget() {
      return target.getBestCameraToTarget();
    }

    @Override
    public double getPoseAmbiguity() {
      return target.getPoseAmbiguity();
    }

    AprilTagPhotonTarget(PhotonTrackedTarget target) {
      super(target);
    }
  }
}
