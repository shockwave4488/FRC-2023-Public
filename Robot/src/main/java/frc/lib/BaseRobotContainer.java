package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.commands.CommandLogger;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import java.util.ArrayList;

public abstract class BaseRobotContainer implements IRobotContainer {
  protected final ArrayList<ShockwaveSubsystemBase> subsystems;

  /* PreferencesParser & Logger objects are present in this class to remind people to pass them into
  RobotContainer constructors (which is why they aren't used here and the SpotBugs bypass below
  is used) */

  @edu.umd.cs.findbugs.annotations.SuppressFBWarnings(
      value = "URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD",
      justification = "Used in dependent classes")
  protected PreferencesParser prefs;

  @edu.umd.cs.findbugs.annotations.SuppressFBWarnings(
      value = "URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD",
      justification = "Used in dependent classes")
  protected LogManager logger;

  protected BaseRobotContainer(PreferencesParser prefs, LogManager logger) {
    subsystems = new ArrayList<>();
    this.prefs = prefs;
    this.logger = logger;
  }

  @Override
  public void runOnStart() {
    subsystems.forEach((s) -> s.onStart());
  }

  @Override
  public void runOnStop() {
    subsystems.forEach((s) -> s.onStop());
  }

  @Override
  public void runZeroSensors() {
    subsystems.forEach((s) -> s.zeroSensors());
  }

  @Override
  public void runUpdateSmartDashboard() {
    subsystems.forEach((s) -> s.updateSmartDashboard());
  }

  @Override
  public void runSetUpTrackables(LogManager logger) {
    subsystems.forEach((s) -> s.setUpTrackables(logger));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing a
   * BooleanSupplier of the button state to a {@link edu.wpi.first.wpilibj2.command.button.Trigger
   * Trigger}.
   *
   * <p>Should be called after {@link #addSubsystems()}.
   */
  protected void configureButtonBindings() {
    for (ShockwaveSubsystemBase subsystem : subsystems) {
      if (!subsystem.childRequirements.isEmpty()) {
        Command defaultCommand =
            new RunCommand(() -> {}, subsystem)
                .ignoringDisable(true)
                .handleInterrupt(
                    () -> {
                      if (!DriverStation.isTest())
                        throw new RuntimeException(
                            String.format(
                                """
                A command that requires the %s subsystem has been scheduled! \
                This subsystem has child requirements, so this could potentially \
                be a bug if the command was supposed to interrupt commands using \
                the child requirements. If this is the case, the command should \
                instead require ShockwaveSubsystemBase.childRequirements.""",
                                subsystem.getName()));
                    });
        CommandLogger.getInstance().excludeCommand(defaultCommand);
        subsystem.setDefaultCommand(defaultCommand);
      }
    }
  }

  /**
   * Add each subsystem (each class that extends ShockwaveSubsystemBase) to the ArrayList created in
   * BaseRobotContainer
   */
  protected abstract void addSubsystems();
}
