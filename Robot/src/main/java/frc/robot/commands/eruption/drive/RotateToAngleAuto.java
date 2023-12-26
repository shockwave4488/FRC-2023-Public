package frc.robot.commands.eruption.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.sensors.gyro.NavX;
import frc.robot.subsystems.drive.SwerveDrive;

/** Eruption's command for rotating to an angle during autonomous. */
public class RotateToAngleAuto extends CommandBase {
  private final SwerveDrive swerve;
  private static final double TURN_P = 0.1;
  private static final double TURN_I = 0.001;
  private static final double TURN_D = 0;
  private PIDController turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
  private double desiredAngle;
  private NavX gyro;
  private double currentAngle;
  private static final double ANGLE_RANGE = 4;

  public boolean pidAtSetpoint() {
    return turnPID.atSetpoint();
  }

  public RotateToAngleAuto(SwerveDrive swerve, NavX gyro, double desiredAngle) {
    this.swerve = swerve;
    addRequirements(swerve.rotationRequirement);

    this.desiredAngle = desiredAngle;
    this.gyro = gyro;

    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(ANGLE_RANGE);
  }

  @Override
  public void execute() {
    currentAngle = gyro.getYaw().getDegrees();
    turnPID.setSetpoint(desiredAngle);
    double rotSpeed = turnPID.calculate(currentAngle);
    swerve.setRotationSpeed(rotSpeed);

    LeveledSmartDashboard.INFO.putNumber("Current Angle", currentAngle);
    LeveledSmartDashboard.INFO.putNumber("Desired Angle", desiredAngle);
  }
}
