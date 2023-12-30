package frc.lib.wpiextensions;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LogManager;
import java.util.ArrayList;
import java.util.List;

/** Insert comment on what exactly this class does */
public abstract class ShockwaveSubsystemBase extends SubsystemBase {
  public List<Subsystem> childRequirements = new ArrayList<>();

  /**
   * What the subsystem should do upon starting up, replacement of onStart() from our old Loop class
   */
  public abstract void onStart();

  /**
   * What the subsystem should do upon shutting down, replacement of onStop() from our old Loop
   * class
   */
  public abstract void onStop();

  /** Where all sensors should be zeroed/reset */
  public abstract void zeroSensors();

  /** Where you should update all desired values to Smart Dashboard */
  public abstract void updateSmartDashboard();

  /** Where you should set up the trackables to be logged */
  public abstract void setUpTrackables(LogManager logger);

  /**
   * Makes a dummy subsystem that can act as a lock on part of this subsystem's functionality. If
   * you require the parent subsystem, the child requirements are <b>not</b> automatically added
   * too, so commands using those requirements will not be interrupted. If this is the desired
   * behavior, pass the command requiring the whole subsystem into {@link
   * frc.lib.commands.CommandUtil#withChildRequirements(edu.wpi.first.wpilibj2.command.CommandBase)
   * withChildRequirements(CommandBase)}.
   */
  protected SubsystemBase childRequirement() {
    SubsystemBase childRequirement = new SubsystemBase() {};
    childRequirements.add(childRequirement);
    return childRequirement;
  }
}
