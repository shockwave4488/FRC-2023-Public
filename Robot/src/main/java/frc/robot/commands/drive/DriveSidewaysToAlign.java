package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.autonomous.AutoPIDControllerContainer;
import frc.lib.controlsystems.SupplierCache;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

/**
 * Moves the robot along the shortest path (based on pose) to be in line with a target. Designed to
 * be executed in parallel with a command that attempts to align to the target.
 *
 * <p>Note: we should consider making the end condition dependent on pose so this command can be
 * used in other contexts.
 */
public class DriveSidewaysToAlign extends CommandBase {
  private static final Rotation2d oppositeAngleRotation = Rotation2d.fromRotations(0.5);

  private final SwerveDrive swerve;
  private final PIDController xController;
  private final PIDController yController;

  public final SupplierCache<Pose2d> targetPose;
  private Rotation2d curTargetAngle;

  public DriveSidewaysToAlign(
      SwerveDrive swerve, AutoPIDControllerContainer pidControllers, Supplier<Pose2d> targetPose) {
    this.swerve = swerve;
    xController = pidControllers.xPidController;
    yController = pidControllers.yPidController;
    this.targetPose = new SupplierCache<>(targetPose, this::updateAngle);
    addRequirements(swerve.driveRequirement);
  }

  public Rotation2d getTargetAngle() {
    return curTargetAngle;
  }

  private void updateAngle() {
    xController.reset();
    yController.reset();
    // Robot should be pointing 180 degrees from the direction the target faces
    curTargetAngle = targetPose.current().getRotation().rotateBy(oppositeAngleRotation);
  }

  @Override
  public void initialize() {
    targetPose.update();
  }

  @Override
  public void execute() {
    Translation2d posDiff =
        swerve.getOdometry().getTranslation().minus(targetPose.current().getTranslation());
    Rotation2d angleDiff = posDiff.getAngle().minus(targetPose.current().getRotation());
    Translation2d distToAlign =
        new Translation2d(posDiff.getNorm() * angleDiff.getSin(), curTargetAngle);

    // We feed the error rather than a measurement, so setpoint should be zero
    // X and Y reversed to account for differing coordinate systems
    double xSpeed = xController.calculate(distToAlign.getY());
    double ySpeed = yController.calculate(-distToAlign.getX());
    swerve.setTranslationSpeeds(xSpeed, ySpeed, true);
  }
}
