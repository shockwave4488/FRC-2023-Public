package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.DoubleSupplier;

public class StandardRotation extends CommandBase {
  private final SwerveDrive swerve;
  private final DoubleSupplier rotSupplier;

  public StandardRotation(SwerveDrive swerve, double rotationMultiplier, DoubleSupplier rot) {
    this.swerve = swerve;
    rotSupplier = () -> rot.getAsDouble() * rotationMultiplier;
    addRequirements(swerve.rotationRequirement);
  }

  @Override
  public void execute() {
    swerve.setRotationSpeed(rotSupplier.getAsDouble());
  }
}
