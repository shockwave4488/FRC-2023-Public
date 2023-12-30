package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.commands.CommandComposer;
import frc.lib.controlsystems.SupplierCache;
import frc.lib.sensors.gyro.NavX;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class RotateTowardsPosition extends CommandComposer<RotateToAngle> {
  private static Rotation2d getAngleToPosition(SwerveDrive swerve, Translation2d position) {
    return position.minus(swerve.getOdometry().getTranslation()).getAngle();
  }

  public final SupplierCache<Translation2d> targetPos;

  public RotateTowardsPosition(
      SwerveDrive swerve,
      NavX gyro,
      ProfiledPIDController thetaController,
      Supplier<Translation2d> position) {
    targetPos = new SupplierCache<>(position);
    targetPos.update();
    setComposedCommand(
        new RotateToAngle(
            swerve, gyro, thetaController, () -> getAngleToPosition(swerve, targetPos.current())));
  }

  @Override
  public void initialize() {
    targetPos.update();
    super.initialize();
  }

  @Override
  public void execute() {
    composedCommand.targetAngle.update();
    super.execute();
  }
}
