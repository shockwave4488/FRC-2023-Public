package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.lib.controlsystems.DoneCycleMachineConditions.NumberCondition;
import frc.lib.controlsystems.SupplierCache;
import frc.lib.sensors.gyro.NavX;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class RotateToAngle extends CommandBase {
  private static final Rotation2d DEFAULT_ANGLE_RANGE = Rotation2d.fromDegrees(4);

  private final SwerveDrive swerve;
  private final NavX gyro;
  private ProfiledPIDController thetaController;

  public final SupplierCache<Rotation2d> targetAngle;

  public RotateToAngle(
      SwerveDrive swerve,
      NavX gyro,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredAngle) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.thetaController = thetaController;
    targetAngle = new SupplierCache<>(desiredAngle);
    targetAngle.update();
    addRequirements(swerve.rotationRequirement);
  }

  public DoneCycleMachine<NumberCondition> getYawDoneCycleMachine(
      int minDoneCycles, Rotation2d angleDoneRange) {
    return DoneCycleMachine.withMinCycles(
            new NumberCondition(
                () -> gyro.getYaw().getRadians(), () -> targetAngle.current().getRadians()),
            minDoneCycles)
        .configureCondition(
            condition -> condition.doneRange.setTolerance(angleDoneRange.getRadians()));
  }

  public DoneCycleMachine<NumberCondition> getYawDoneCycleMachine(int minDoneCycles) {
    return getYawDoneCycleMachine(minDoneCycles, DEFAULT_ANGLE_RANGE);
  }

  @Override
  public void initialize() {
    thetaController.reset(gyro.getYaw().getRadians(), gyro.getYawRateRadiansPerSec());
    targetAngle.update();
  }

  @Override
  public void execute() {
    double currentAngle = gyro.getYaw().getRadians();
    double rotSpeed = thetaController.calculate(currentAngle, targetAngle.current().getRadians());
    swerve.setRotationSpeed(rotSpeed);
  }
}
