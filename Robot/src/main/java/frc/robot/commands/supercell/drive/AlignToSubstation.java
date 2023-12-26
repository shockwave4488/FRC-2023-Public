package frc.robot.commands.supercell.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.commands.CommandUtil;
import frc.lib.commands.LogCommand;
import frc.lib.misc.Util;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc.robot.commands.drive.DriveRelativeToAprilTag;
import frc.robot.commands.drive.DriveRelativeToAprilTag.RelativeGoalPose;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc.robot.commands.supercell.arm.MoveArmWithPID;
import frc.robot.constants.Constants.DriveTrainConstants;
import frc.robot.constants.Constants2023.DoubleSubstationSide;
import frc.robot.constants.Constants2023.FieldConstants;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc.robot.constants.Constants2023.RobotConstants.IntakeConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.supercell.Arm;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class AlignToSubstation {
  private AlignToSubstation() {}

  public static Command from(
      SwerveDrive swerve,
      NavX gyro,
      AprilTagCamera camera,
      Arm arm,
      AutoPIDControllerContainer pidControllers,
      Supplier<DoubleSubstationSide> substationSide) {
    AtomicReference<AprilTagTarget> target = new AtomicReference<>();
    return CommandUtil.withName(
        LogCommand.parallel(
            LogCommand.sequence(
                new WaitUntilCommand(
                    () -> {
                      Optional<? extends AprilTagTarget> possibleTarget =
                          camera.getBestAprilTagTarget();
                      if (possibleTarget.isPresent()
                          && possibleTarget.get().getId()
                              == FieldConstants.getInstance().substationTagId) {
                        target.set(possibleTarget.get());
                        return true;
                      }
                      return false;
                    }),
                LogCommand.proxy(
                    new DriveRelativeToAprilTag(
                        swerve,
                        pidControllers,
                        Util.constantSupplierOf(
                            new TrajectoryConfig(
                                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED / 2,
                                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL / 2)),
                        () ->
                            target
                                .get()
                                .toAprilTag(
                                    swerve.getOdometry(),
                                    camera.getCameraPositionConsts().robotCenterToCamera),
                        RelativeGoalPose.identity()
                            .translate(
                                () ->
                                    new Translation2d(
                                        -(ArmConstants.ARM_LENGTH
                                                + Units.inchesToMeters(
                                                    IntakeConstants.EXTENSION_LENGTH))
                                            * Math.cos(ArmSetpoint.SUBSTATION.angleRadians),
                                        substationSide.get().offsetToSideCenter))
                            .withRotation(Util.constantSupplierOf(new Rotation2d())))),
                LogCommand.proxy(
                    LogCommand.parallel(
                        new RotateToAngle(
                            swerve,
                            gyro,
                            pidControllers.thetaPidController,
                            () -> new Rotation2d()),
                        new StartEndCommand(
                            () -> swerve.setModifier(SwerveModifier.forSpeed(0.5)),
                            swerve::clearModifier,
                            swerve.modifierRequirement)))),
            new MoveArmWithPID(arm, ArmSetpoint.SUBSTATION)),
        "AlignToSubstation");
  }
}
