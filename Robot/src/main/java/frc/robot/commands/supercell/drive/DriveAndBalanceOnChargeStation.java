package frc.robot.commands.supercell.drive;

import static frc.robot.commands.supercell.drive.BalanceOnChargeStation.driveAtHorizontalSpeed;
import static frc.robot.commands.supercell.drive.BalanceOnChargeStation.getDriveDirection;
import static frc.robot.commands.supercell.drive.BalanceOnChargeStation.getTilt;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.commands.CommandUtil;
import frc.lib.commands.LogCommand;
import frc.lib.controlsystems.SupplierCache;
import frc.lib.sensors.gyro.NavX;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.constants.Constants2023.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BiPredicate;
import java.util.function.Supplier;

public class DriveAndBalanceOnChargeStation {
  private DriveAndBalanceOnChargeStation() {}

  public static class ApproachBehavior {
    public static ApproachBehavior fromVelocity(
        SwerveDrive swerve, Supplier<Double> approachSpeed) {
      SupplierCache<Double> metersPerSecond = new SupplierCache<>(approachSpeed);
      return new ApproachBehavior(
              driveAtHorizontalSpeed(swerve, metersPerSecond::current),
              (gyro, gyroAdjustment) ->
                  hasClimbingStarted(gyro, gyroAdjustment, metersPerSecond.current()))
          .withSetup(metersPerSecond::update);
    }

    public static ApproachBehavior fromSpeedWithPoseDirection(
        SwerveDrive swerve, double approachSpeed) {
      return fromVelocity(
          swerve,
          () ->
              Math.copySign(
                  approachSpeed,
                  FieldConstants.CHARGE_STATION_CENTER_X - swerve.getOdometry().getX()));
    }

    public ApproachBehavior withDelayTime(
        SwerveDrive swerve, NavX gyro, double approachSpeed, double durationSeconds) {
      AtomicInteger direction = new AtomicInteger();
      return afterClimbingDo(
          LogCommand.sequence(
              Commands.runOnce(
                  () -> direction.set(getDriveDirection(gyro, swerve.getGyroAdjustment()))),
              LogCommand.endAfter(
                  driveAtHorizontalSpeed(swerve, () -> direction.get() * approachSpeed),
                  durationSeconds)));
    }

    public ApproachBehavior withTimeout(double timeoutSeconds) {
      return terminateEarlyAfter(new WaitCommand(timeoutSeconds));
    }

    public ApproachBehavior withRotation(
        SwerveDrive swerve, NavX gyro, ProfiledPIDController thetaController, Rotation2d rotation) {
      return runWhileApproaching(new RotateToAngle(swerve, gyro, thetaController, () -> rotation));
    }

    public BiPredicate<NavX, Rotation2d> eitherSide() {
      return (gyro, gyroAdjustment) ->
          Math.abs(getTilt(gyro, gyroAdjustment).getRadians())
              > CLIMBING_CHARGE_STATION_ANGLE.getRadians();
    }

    public final CommandBase approachCommand;
    public final BiPredicate<NavX, Rotation2d> shouldStartClimbing;
    public Runnable setup = () -> {};
    public Command afterClimbing = Commands.none();
    public Command terminateEarlyCommand = CommandUtil.indefiniteInstantCommand(() -> {});
    public Command approachParallelCommand = Commands.none();

    public ApproachBehavior(
        CommandBase approachCommand, BiPredicate<NavX, Rotation2d> shouldStartClimbing) {
      this.approachCommand = approachCommand;
      this.shouldStartClimbing = shouldStartClimbing;
    }

    public ApproachBehavior withSetup(Runnable setup) {
      this.setup = setup;
      return this;
    }

    public ApproachBehavior afterClimbingDo(Command command) {
      afterClimbing = command;
      return this;
    }

    public ApproachBehavior terminateEarlyAfter(Command command) {
      terminateEarlyCommand = command;
      return this;
    }

    public ApproachBehavior runWhileApproaching(Command command) {
      approachParallelCommand = command;
      return this;
    }
  }

  private static final Rotation2d CLIMBING_CHARGE_STATION_ANGLE = Rotation2d.fromDegrees(10);

  private static boolean hasClimbingStarted(
      NavX gyro, Rotation2d gyroAdjustment, int chargeStationDirection) {
    return getTilt(gyro, gyroAdjustment)
                .minus(CLIMBING_CHARGE_STATION_ANGLE.times(-chargeStationDirection))
                .getRadians()
            * chargeStationDirection
        < 0;
  }

  private static boolean hasClimbingStarted(
      NavX gyro, Rotation2d gyroAdjustment, double approachVelocity) {
    return hasClimbingStarted(gyro, gyroAdjustment, (int) Math.signum(approachVelocity));
  }

  public static CommandBase create(SwerveDrive swerve, NavX gyro, ApproachBehavior behavior) {
    AtomicBoolean terminateEarly = new AtomicBoolean();
    return LogCommand.sequence(
        CommandUtil.withName(
            Commands.runOnce(
                () -> {
                  terminateEarly.set(false);
                  behavior.setup.run();
                }),
            "Setup"),
        LogCommand.endWhen(
            LogCommand.sequence(
                LogCommand.deadline(
                    LogCommand.sequence(
                        LogCommand.race(
                            LogCommand.endWhen(
                                CommandUtil.withName(behavior.approachCommand, "ApproachStation"),
                                () ->
                                    behavior.shouldStartClimbing.test(
                                        gyro, swerve.getGyroAdjustment())),
                            LogCommand.sequence(
                                behavior.terminateEarlyCommand,
                                Commands.runOnce(() -> terminateEarly.set(true)))),
                        behavior.afterClimbing),
                    behavior.approachParallelCommand),
                CommandUtil.withName(BalanceOnChargeStation.create(swerve, gyro), "Balance")),
            terminateEarly::get));
  }
}
