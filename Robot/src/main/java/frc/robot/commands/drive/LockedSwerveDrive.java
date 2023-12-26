package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.SwerveParameters.ModulePosition;
import frc.lib.logging.LeveledSmartDashboard;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.Map;

public class LockedSwerveDrive extends CommandBase {
  public enum LockedMode {
    Stop,
    XShape,
    Octagon,
    Offset
  }

  private final SwerveDrive swerve;
  private LockedMode mode;
  private boolean forceMode = true;
  private final SendableChooser<LockedMode> modeSelector = new SendableChooser<>();

  /**
   * Drive setting preventing driver input, while locking robot in place.
   *
   * @param swerve SwerveDrive subsystem
   * @param mode Wheel behavior mode
   *     <ul>
   *       <li>Stop = Hold current position
   *       <li>XShape = Hold wheel cross position (wheels pointed to center)
   *       <li>Octagon = Hold octagon position (opposite of XShape)
   *       <li>Offset = Hold position with wheels 45 degrees off from each other
   *     </ul>
   */
  public LockedSwerveDrive(SwerveDrive swerve, LockedMode mode) {
    this.swerve = swerve;
    this.mode = mode;
    addRequirements(swerve.driveRequirement, swerve.rotationRequirement);
  }

  public LockedSwerveDrive(SwerveDrive swerve) {
    this(swerve, LockedMode.Stop);
  }

  public LockedSwerveDrive(SwerveDrive swerve, String selectorName) {
    this(swerve);
    forceMode = false;
    modeSelector.setDefaultOption("Current position", LockedMode.Stop);
    modeSelector.addOption("X-position", LockedMode.XShape);
    modeSelector.addOption("Offset positions", LockedMode.Offset);
    modeSelector.addOption("Octagon", LockedMode.Octagon);
    LeveledSmartDashboard.HIGH.putData("LockedSwerveDrive " + selectorName, modeSelector);
  }

  @Override
  public void initialize() {
    if (!forceMode) {
      mode = modeSelector.getSelected();
    }

    switch (mode) {
      case XShape:
        swerve.assignModuleStates(
            Map.of(
                ModulePosition.FRONT_LEFT, new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                ModulePosition.FRONT_RIGHT, new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                ModulePosition.BACK_LEFT, new SwerveModuleState(0, new Rotation2d(Math.PI * 3 / 4)),
                ModulePosition.BACK_RIGHT,
                    new SwerveModuleState(0, new Rotation2d(-Math.PI * 3 / 4))));
        break;
      case Octagon:
        swerve.assignModuleStates(
            Map.of(
                ModulePosition.FRONT_LEFT, new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                ModulePosition.FRONT_RIGHT, new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                ModulePosition.BACK_LEFT,
                    new SwerveModuleState(0, new Rotation2d(-Math.PI * 3 / 4)),
                ModulePosition.BACK_RIGHT,
                    new SwerveModuleState(0, new Rotation2d(Math.PI * 3 / 4))));
        break;
      case Offset:
        swerve.assignModuleStates(
            Map.of(
                ModulePosition.FRONT_LEFT, new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                ModulePosition.FRONT_RIGHT, new SwerveModuleState(0, new Rotation2d(0)),
                ModulePosition.BACK_LEFT, new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)),
                ModulePosition.BACK_RIGHT,
                    new SwerveModuleState(0, new Rotation2d(-Math.PI * 3 / 4))));
        break;
      case Stop:
      default:
        swerve.stop();
        break;
    }
  }
}
