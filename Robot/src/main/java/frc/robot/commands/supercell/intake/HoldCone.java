package frc.robot.commands.supercell.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.commands.CommandUtil;
import frc.lib.preferences.PreferencesParser;
import frc.robot.subsystems.supercell.Intake;

public class HoldCone {
  private HoldCone() {}

  public static Command oscillate(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(Intake.Speed.HOLD), intake)
        .andThen(new WaitCommand(0.6))
        .andThen(
            new InstantCommand(() -> intake.setSpeed(Intake.Speed.REV_HOLD), intake)
                .andThen(new WaitCommand(0.05)))
        .repeatedly()
        .ignoringDisable(true)
        .withName("HoldCone");
  }

  public static Command stall(Intake intake) {
    return CommandUtil.indefiniteInstantCommand(() -> intake.setSpeed(Intake.Speed.HOLD), intake)
        .ignoringDisable(true)
        .withName("HoldCone");
  }

  public static Command basedOnPrefs(Intake intake, PreferencesParser prefs) {
    return prefs.tryGetValue(prefs::getBoolean, "HoldConeStall", false)
        ? stall(intake)
        : oscillate(intake);
  }
}
