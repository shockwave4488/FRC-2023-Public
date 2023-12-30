package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.flowcontrol.EdgeTrigger;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class SwerveModifierCommand extends CommandBase {

  public static record SwerveModifier(
      double moveScale, double rotationScale, Translation2d centerOffset) {
    public static SwerveModifier forSpeed(double moveScale, double rotationSpeed) {
      return new SwerveModifier(moveScale, rotationSpeed, new Translation2d());
    }

    public static SwerveModifier forSpeed(double scale) {
      return forSpeed(scale, scale);
    }

    public static SwerveModifier forCenterOffset(Translation2d offset) {
      return new SwerveModifier(1, 1, offset);
    }

    public static SwerveModifier forCenterOffset(double x, double y) {
      return forCenterOffset(new Translation2d(x, y));
    }

    public static SwerveModifier forSwerveModule(SwerveModule module) {
      return forCenterOffset(module.getLocation());
    }

    public static SwerveModifier forNone() {
      return new SwerveModifier(1, 1, new Translation2d());
    }

    public SwerveModifier with(SwerveModifier other) {
      return new SwerveModifier(
          moveScale * other.moveScale,
          rotationScale * other.rotationScale,
          centerOffset.plus(other.centerOffset));
    }

    public SwerveModifier rotateBy(Rotation2d rot) {
      return new SwerveModifier(moveScale, rotationScale, centerOffset.rotateBy(rot));
    }
  }

  private final Map<BooleanSupplier, SwerveModifier> modifiers = new HashMap<>();
  private boolean offsetStarted; // Freezes the axis of rotation while rotating
  private Rotation2d startRotation;
  private final SwerveDrive swerve;

  public SwerveModifierCommand(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve.modifierRequirement);
  }

  public SwerveModifierCommand bindModifier(BooleanSupplier trigger, SwerveModifier modifier) {
    modifiers.put(trigger, modifier);
    return this;
  }

  public SwerveModifierCommand bindModifierToggle(
      BooleanSupplier trigger, SwerveModifier modifier) {
    return bindModifier(
        new BooleanSupplier() {
          private final EdgeTrigger edge = new EdgeTrigger();
          private boolean toggle;

          @Override
          public boolean getAsBoolean() {
            if (edge.getRisingUpdate(trigger.getAsBoolean())) {
              toggle = !toggle;
            }
            return toggle;
          }
        },
        modifier);
  }

  private SwerveModifier calculateModifiers() {
    SwerveModifier output = SwerveModifier.forNone();
    for (Map.Entry<BooleanSupplier, SwerveModifier> modifier : modifiers.entrySet()) {
      if (modifier.getKey().getAsBoolean()) {
        output = output.with(modifier.getValue());
      }
    }
    if (swerve.isFieldRelative()) {
      if (output.centerOffset().getX() == 0 && output.centerOffset().getY() == 0) {
        offsetStarted = false;
      } else {
        if (offsetStarted) {
          output = output.rotateBy(swerve.getGyroYaw().minus(startRotation));
        } else {
          offsetStarted = true;
          startRotation = swerve.getGyroYaw();
        }
      }
    } else {
      offsetStarted = false;
    }
    return output;
  }

  @Override
  public void execute() {
    swerve.setModifier(calculateModifiers());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.clearModifier();
  }
}
