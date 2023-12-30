package frc.robot.commands.supercell.drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.commands.LogCommand;
import frc.lib.sensors.gyro.NavX;
import frc.robot.Robot;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc.robot.constants.Constants2023.RobotConstants.AutonomousConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.ArrayDeque;
import java.util.function.DoubleSupplier;

public class BalanceOnChargeStation {
  /**
   * Gets the tilt of the robot on an inclined plane by combining pitch and roll information.
   * Negative when slanted upwards, regardless of yaw.
   *
   * @param zeroRollAngle Yaw at which driving up the inclined plane will result in no roll.
   */
  public static Rotation2d getTilt(NavX gyro, Rotation2d zeroRollAngle, Rotation2d gyroAdjustment) {
    Rotation2d angleOffset = gyro.getYaw().plus(gyroAdjustment).minus(zeroRollAngle);
    return gyro.getPitch()
        .times(angleOffset.getCos())
        .plus(gyro.getRoll().times(angleOffset.getSin()));
  }

  public static Rotation2d getTilt(NavX gyro, Rotation2d gyroAdjustment) {
    return getTilt(
        gyro, new Rotation2d(), gyroAdjustment); // Charge station is parallel with the y-axis
  }

  private static boolean isBalanced(NavX gyro, Rotation2d gyroAdjustment) {
    return Math.abs(getTilt(gyro, gyroAdjustment).getRadians()) < BALANCED_RANGE.getRadians();
  }

  public static CommandBase driveAtHorizontalSpeed(
      SwerveDrive swerve, DoubleSupplier metersPerSecond) {
    return Commands.run(
        () -> swerve.setTranslationSpeeds(metersPerSecond.getAsDouble(), 0, true),
        swerve.driveRequirement);
  }

  public static int getDriveDirection(NavX gyro, Rotation2d gyroAdjustment) {
    // By not passing in 180 degrees even if on the other side of the charge station, we can use
    // the tilt (negative next to the driver station, positive on the other side) to decide
    // which way to drive.
    return (int) -Math.signum(getTilt(gyro, gyroAdjustment).getDegrees());
  }

  private static final Rotation2d BALANCED_RANGE = Rotation2d.fromDegrees(2.5);
  private static final double CLIMB_VELOCITY = 0.8;

  public static Command create(SwerveDrive swerve, NavX gyro) {
    return LogCommand.repeat(
        LogCommand.sequence(
            LogCommand.endWhen(
                getTiltPIDCommand(swerve, gyro),
                () -> isBalanced(gyro, swerve.getGyroAdjustment())),
            LogCommand.endWhen(
                new LockedSwerveDrive(swerve, LockedMode.XShape),
                () -> !isBalanced(gyro, swerve.getGyroAdjustment()))));
  }

  private static Command getTiltPIDCommand(SwerveDrive swerve, NavX gyro) {
    return new PIDCommand(
        new PIDController(
            AutonomousConstants.AUTO_BALANCE_P,
            AutonomousConstants.AUTO_BALANCE_I,
            AutonomousConstants.AUTO_BALANCE_D),
        () -> getTilt(gyro, swerve.getGyroAdjustment()).getDegrees(),
        0,
        value -> swerve.setTranslationSpeeds(value, 0, true),
        swerve.driveRequirement);
  }

  private static class AnticipatoryThresholdBalance extends CommandBase {
    private static final double FINALIZE_VELOCITY = 0.01;
    private static final int FINALIZE_LIMIT = 1500;
    private final SwerveDrive swerve;
    private final NavX gyro;
    private int moveDir;
    private double lastTilt;
    private boolean finalize;
    private long finalizeStart;

    public AnticipatoryThresholdBalance(SwerveDrive swerve, NavX gyro) {
      this.swerve = swerve;
      this.gyro = gyro;
      addRequirements(swerve.driveRequirement);
    }

    @Override
    public void initialize() {
      this.lastTilt = getTilt(gyro, swerve.getGyroAdjustment()).getRadians();
      this.moveDir = (lastTilt > 0 ? -1 : 1);
      this.finalize = false;
    }

    @Override
    public void execute() {
      double tilt = getTilt(gyro, swerve.getGyroAdjustment()).getRadians();
      double absTilt = Math.abs(tilt);
      if (!finalize && lastTilt - absTilt > Math.toRadians(5)) {
        finalize = true;
        finalizeStart = System.currentTimeMillis();
        Robot.getConsole().println("Finalizing");
      } else if (lastTilt < absTilt) {
        lastTilt = absTilt;
      }
      if (absTilt > Math.toRadians(8) && (moveDir == 1) != (tilt < 0)) {
        finalize = false;
        moveDir = (tilt > 0 ? -1 : 1);
        lastTilt = absTilt;
        Robot.getConsole().println("Reset: Tilt");
      }
      if (finalize) {
        swerve.setTranslationSpeeds(-moveDir * FINALIZE_VELOCITY * absTilt, 0, true);
        if (finalizeStart + FINALIZE_LIMIT < System.currentTimeMillis()) {
          finalize = false;
          moveDir = (tilt > 0 ? -1 : 1);
          lastTilt = absTilt;
          Robot.getConsole().println("Reset: Timeout");
        }
      } else {
        swerve.setTranslationSpeeds(moveDir * CLIMB_VELOCITY, 0, true);
      }
    }
  }

  private static class AnticipatoryBalance extends CommandBase {
    private final SwerveDrive swerve;
    private final NavX gyro;
    private final PIDController tiltPidController = new PIDController(0.1, 0, 0);
    private final PIDController rateOfTiltChangePidController =
        new PIDController(-(tiltPidController.getP() / 3) / 5, 0, 0);
    private static final int SAMPLE_SIZE = 3;
    private final ArrayDeque<Pair<Double, Double>> prevTilts = new ArrayDeque<>(SAMPLE_SIZE);

    public AnticipatoryBalance(SwerveDrive swerve, NavX gyro) {
      this.swerve = swerve;
      this.gyro = gyro;
      tiltPidController.setSetpoint(0);
      rateOfTiltChangePidController.setSetpoint(0);
      addRequirements(swerve.driveRequirement);
    }

    @Override
    public void initialize() {
      tiltPidController.reset();
      prevTilts.clear();
    }

    @Override
    public void execute() {
      double driveSpeed =
          tiltPidController.calculate(getTilt(gyro, swerve.getGyroAdjustment()).getDegrees());

      Pair<Double, Double> currentTilt =
          Pair.of(
              getTilt(gyro, swerve.getGyroAdjustment()).getDegrees(), gyro.lastTimestampSeconds());
      if (prevTilts.size() == SAMPLE_SIZE) {
        Pair<Double, Double> pastTilt = prevTilts.pop();
        driveSpeed +=
            rateOfTiltChangePidController.calculate(
                (currentTilt.getFirst() - pastTilt.getFirst())
                    / (currentTilt.getSecond() - pastTilt.getSecond()));
      }
      prevTilts.add(currentTilt);

      swerve.setTranslationSpeeds(driveSpeed, 0, true);
    }
  }
}
