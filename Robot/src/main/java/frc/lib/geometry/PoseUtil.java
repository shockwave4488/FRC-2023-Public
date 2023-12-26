package frc.lib.geometry;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import java.util.function.Function;
import java.util.function.Supplier;

public class PoseUtil {
  private PoseUtil() {}

  public interface PoseSupplier extends Supplier<Pose2d> {
    static PoseSupplier fromParts(Supplier<Translation2d> position, Supplier<Rotation2d> rotation) {
      return () -> new Pose2d(position.get(), rotation.get());
    }

    default PoseSupplier withRotation(Supplier<Rotation2d> rotation) {
      return fromParts(() -> get().getTranslation(), rotation);
    }

    default PoseSupplier withTranslation(Supplier<Translation2d> position) {
      return fromParts(position, () -> get().getRotation());
    }

    default PoseSupplier map(Function<Pose2d, Pose2d> mapper) {
      return () -> mapper.apply(get());
    }

    default PoseSupplier withAdjustment(Supplier<Transform2d> adjustment) {
      return () -> get().transformBy(adjustment.get());
    }
  }

  public static Function<Pose2d, Pose2d> adjustmentApplier(Supplier<Transform2d> adjustment) {
    return pose -> pose.transformBy(adjustment.get());
  }

  public static class DualPose extends Pair<Pose2d, Pose2d> {
    public DualPose(Pose2d pose1, Pose2d pose2) {
      super(pose1, pose2);
    }

    /**
     * @return the angle where the first pose points toward the second pose
     */
    public Rotation2d faceAngle() {
      return getSecond().getTranslation().minus(getFirst().getTranslation()).getAngle();
    }

    /**
     * @return the angle where the second pose points toward the first pose
     */
    public Rotation2d faceReverseAngle() {
      return faceAngle().rotateBy(Rotation2d.fromRotations(0.5));
    }

    public <T> T getFromFirst(Function<Pose2d, T> poseFunction) {
      return poseFunction.apply(getFirst());
    }

    public <T> T getFromSecond(Function<Pose2d, T> poseFunction) {
      return poseFunction.apply(getSecond());
    }
  }

  public static Pose2d addTranslation(Pose2d current, Translation2d addition) {
    Rotation2d currentRotation = current.getRotation();
    return new Pose2d(current.getTranslation().plus(addition), currentRotation);
  }

  public static PoseSupplier aprilTagPoseSupplier(Supplier<AprilTag> tag) {
    return () -> tag.get().pose.toPose2d();
  }

  public static PoseSupplier aprilTagPoseSupplier(
      Supplier<AprilTagTarget> target,
      Supplier<Pose2d> robotPose,
      Transform3d robotCenterToCamera) {
    return aprilTagPoseSupplier(
        () -> getAprilTagFromTarget(target.get(), robotPose.get(), robotCenterToCamera));
  }

  public static AprilTag getAprilTagFromTarget(
      AprilTagTarget target, Pose2d robotPose, Transform3d robotCenterToCamera) {
    return target.toAprilTag(robotPose, robotCenterToCamera);
  }

  public static Rotation2d lerpRotationShortest(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.interpolate(endValue, t);
  }

  public static Rotation2d lerpRotationLongest(
      Rotation2d startValue, Rotation2d endValue, double t) {
    double diff = endValue.minus(startValue).getRadians();
    return Rotation2d.fromRadians(
        startValue.getRadians()
            + (diff - Math.copySign(2 * Math.PI, diff)) * MathUtil.clamp(t, 0, 1));
  }

  public static Rotation2d lerpRotationClockwise(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.plus(
        new Rotation2d(
                MathUtil.inputModulus(
                    endValue.getRadians() - startValue.getRadians(), 0, 2 * Math.PI))
            .times(MathUtil.clamp(t, 0, 1)));
  }

  public static Rotation2d lerpRotationCounterclockwise(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.plus(
        new Rotation2d(
                -MathUtil.inputModulus(
                    startValue.getRadians() - endValue.getRadians(), 0, 2 * Math.PI))
            .times(MathUtil.clamp(t, 0, 1)));
  }
}
