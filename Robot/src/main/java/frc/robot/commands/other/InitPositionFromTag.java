package frc.robot.commands.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.robot.subsystems.drive.SwerveDrive;

public class InitPositionFromTag {

  public static Command create(SwerveDrive swerve, NavX gyro, Limelight camera) {
    return new WaitUntilCommand(() -> camera.hasTargets())
        .andThen(
            () -> {
              Pose2d pose = camera.getRobotPose().toPose2d();
              gyro.setYawAdjustment(new Rotation2d());
              gyro.setYawAdjustment(pose.getRotation().minus(gyro.getYaw()));
              swerve.resetOdometry(pose);
            });
  }
}
