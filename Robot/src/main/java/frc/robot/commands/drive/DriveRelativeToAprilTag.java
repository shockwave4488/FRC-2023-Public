package frc.robot.commands.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.CommandComposer;
import frc.lib.controlsystems.SupplierCache;
import frc.lib.geometry.PoseUtil.DualPose;
import frc.lib.misc.Util;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Function;
import java.util.function.Supplier;

/** Command for autonomously driving to a place relative to the position of a still AprilTag */
public class DriveRelativeToAprilTag extends CommandComposer<SwerveDriveToPosition> {
  public static class DriveInfo {
    public final Pose2d startPose;
    public final AprilTag tag;
    public final Pose2d targetPose2d;

    public DriveInfo(Pose2d startPose, AprilTag tag) {
      this.startPose = startPose;
      this.tag = tag;
      targetPose2d = tag.pose.toPose2d();
      Robot.getConsole().debugln("AutoDrive > AprilTag Pose: " + targetPose2d);
    }
  }

  @FunctionalInterface
  public interface RelativeGoalPose {
    public static RelativeGoalPose identity() {
      return info -> info.targetPose2d;
    }

    public Pose2d getGoalPose(DriveInfo info);

    public default RelativeGoalPose translate(Supplier<Translation2d> offset) {
      return info -> {
        Pose2d prev = getGoalPose(info);
        return new Pose2d(prev.getTranslation().plus(offset.get()), prev.getRotation());
      };
    }

    public default RelativeGoalPose translateOffRotation(Supplier<Translation2d> offset) {
      return info -> getGoalPose(info).transformBy(new Transform2d(offset.get(), new Rotation2d()));
    }

    public default RelativeGoalPose withRotation(Function<DriveInfo, Rotation2d> rotation) {
      return info -> {
        Pose2d prev = getGoalPose(info);
        return new Pose2d(prev.getTranslation(), rotation.apply(info));
      };
    }

    public default RelativeGoalPose withRotation(Supplier<Rotation2d> rotation) {
      return withRotation(info -> rotation.get());
    }

    public default RelativeGoalPose faceTarget() {
      return faceTargetWithOffset(Util.constantSupplierOf(new Rotation2d()));
    }

    public default RelativeGoalPose faceTargetWithOffset(Supplier<Rotation2d> offset) {
      return info -> {
        Pose2d prev = getGoalPose(info);
        return new Pose2d(
            prev.getTranslation(),
            new DualPose(prev, info.targetPose2d).faceAngle().plus(offset.get()));
      };
    }

    public default RelativeGoalPose faceOriginalAngle() {
      return withRotation(info -> info.startPose.getRotation());
    }

    public default RelativeGoalPose angleOfTarget() {
      return angleOfTargetWithOffset(Util.constantSupplierOf(new Rotation2d()));
    }

    public default RelativeGoalPose angleOfTargetReversed() {
      return angleOfTargetWithOffset(Util.constantSupplierOf(new Rotation2d(Math.PI)));
    }

    public default RelativeGoalPose angleOfTargetWithOffset(Supplier<Rotation2d> offset) {
      return withRotation(info -> info.targetPose2d.getRotation().plus(offset.get()));
    }
  }

  public final SupplierCache<AprilTag> targetAprilTag;

  /**
   * @param swerve Swerve drive subsystem
   * @param pidControllers PID controllers controlling translation and rotation to use when
   *     following the trajectory.
   * @param config Trajectory configuration
   * @param tag AprilTag to drive to (stored pose must be up-to-date)
   * @param goalPose Transform (from the perspective of the tag) to be added to the calculated tag
   *     coordinates. For example, "Transform2d(Translation2d(2, -1)" would result in the robot
   *     center stopping 2 meters in front of the tag, and 1 meter to the right of the tag (from the
   *     tag's perspective). The rotation part of the transform does not matter, as it is controlled
   *     instead by {@code rotationBehavior}.
   * @param rotationBehavior Function that takes in a {@link DualPose} (transformed robot position
   *     first, target position second), and returns the angle the robot should end at. This class
   *     has a few static factory methods which should cover most rotation behaviors one might want.
   */
  public DriveRelativeToAprilTag(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<TrajectoryConfig> config,
      Supplier<AprilTag> aprilTag,
      RelativeGoalPose goalPose) {
    targetAprilTag = new SupplierCache<>(aprilTag, () -> composedCommand.trajectory.update());
    setComposedCommand(
        new SwerveDriveToPosition(
            swerve,
            pidControllers,
            config,
            () -> {
              return goalPose.getGoalPose(
                  new DriveInfo(swerve.getOdometry(), targetAprilTag.current()));
            },
            Util.constantSupplierOf(new Translation2d[0])));
  }

  @Override
  public void initialize() {
    targetAprilTag.update();
  }
}
