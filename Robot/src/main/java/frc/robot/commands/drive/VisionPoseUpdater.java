package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.math.Average;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionTargets.AprilTagLimelightTarget;
import frc.robot.constants.Constants2023.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionPoseUpdater extends CommandBase {

  private final SwerveDrive swerve;
  private final Supplier<Optional<PoseReference>> pose;
  private final Average<Pose2d> poseAverage;
  private final Field2d field;
  private int ticksSinceUpdate;

  private static record PoseReference(Pose2d robotPose, Pose2d refPose) {}

  public static VisionPoseUpdater createForLimelight(SwerveDrive swerve, Limelight camera) {
    return new VisionPoseUpdater(
        swerve,
        () -> {
          Optional<AprilTagLimelightTarget> bestAprilTag = camera.getBestAprilTagTarget();
          if (bestAprilTag.isEmpty()) {
            return Optional.empty();
          }
          Optional<Pose3d> tagPos =
              FieldConstants.getInstance().tagsOnField.getTagPose(bestAprilTag.get().getId());
          if (tagPos.isEmpty()) {
            return Optional.empty();
          }
          Pose3d robotPose = camera.getRobotPose();
          LeveledSmartDashboard.INFO.putNumber(
              "VisionRobotRotX", Math.toDegrees(robotPose.getRotation().getX()));
          LeveledSmartDashboard.INFO.putNumber(
              "VisionRobotRotY", Math.toDegrees(robotPose.getRotation().getY()));
          LeveledSmartDashboard.INFO.putNumber(
              "VisionRobotRotZ", Math.toDegrees(robotPose.getRotation().getZ()));
          return Optional.of(
              new PoseReference(camera.getRobotPose().toPose2d(), tagPos.get().toPose2d()));
        },
        10);
  }

  private VisionPoseUpdater(
      SwerveDrive swerve, Supplier<Optional<PoseReference>> pose, int averagePoses) {
    this.swerve = swerve;
    this.pose = pose;
    this.poseAverage = Average.createForPose(averagePoses);
    this.field = new Field2d();
    LeveledSmartDashboard.INFO.putData(field);
  }

  @Override
  public void execute() {
    Optional<PoseReference> newPose = pose.get();
    ChassisSpeeds speed = swerve.getRobotRelativeChassisSpeeds();
    if (speed.vxMetersPerSecond == 0
        && speed.vyMetersPerSecond == 0
        && speed.omegaRadiansPerSecond == 0) {
      if (newPose.isPresent()) {
        PoseReference newPoseValue = newPose.get();
        poseAverage.addValue(newPoseValue.robotPose());
        poseAverage.getValue().ifPresent(swerve::consumeVisionEstimate);
      }
    } else {
      poseAverage.reset();
    }

    field.setRobotPose(swerve.getOdometry());
    updateSmartDashboard(newPose);
    ticksSinceUpdate++;
    if (newPose.isPresent()) {
      ticksSinceUpdate = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (field != null) {
      field.close();
    }
  }

  private void updateSmartDashboard(Optional<PoseReference> newPose) {
    Pose2d robotPose = swerve.getOdometry();
    newPose.ifPresent(ref -> field.getObject("Reference").setPose(ref.refPose()));
    LeveledSmartDashboard.INFO.putNumber("TicksSinceVisionUpdate", ticksSinceUpdate);
    LeveledSmartDashboard.INFO.putNumber("RobotX", robotPose.getX());
    LeveledSmartDashboard.INFO.putNumber("RobotY", robotPose.getY());
    LeveledSmartDashboard.INFO.putNumber("RobotYaw", robotPose.getRotation().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("GyroYaw", swerve.getGyroYaw().getDegrees());
  }
}
