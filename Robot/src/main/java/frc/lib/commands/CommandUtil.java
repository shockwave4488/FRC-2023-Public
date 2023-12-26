package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.misc.Util;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import java.util.function.BooleanSupplier;

public class CommandUtil {
  private CommandUtil() {}

  /**
   * Add the child requirements of the specified {@code parentRequirement} subsystem to the {@code
   * command}.
   */
  public static <T extends CommandBase> T withChildRequirementsOf(
      T command, ShockwaveSubsystemBase... parentRequirements) {
    command.addRequirements(parentRequirements);
    for (ShockwaveSubsystemBase parentRequirement : parentRequirements) {
      command.addRequirements(parentRequirement.childRequirements.toArray(Subsystem[]::new));
    }
    return command;
  }

  /** {@link Command#until(BooleanSupplier)} that preserves the original command name. */
  public static ParallelRaceGroup endWhen(Command command, BooleanSupplier condition) {
    ParallelRaceGroup untilCommand = command.until(condition);
    untilCommand.setName(command.getName());
    return untilCommand;
  }

  /** {@link Command#withTimeout(double)} that preserves the original command name. */
  public static ParallelRaceGroup endAfter(Command command, double seconds) {
    ParallelRaceGroup timeoutCommand = command.withTimeout(seconds);
    timeoutCommand.setName(command.getName());
    return timeoutCommand;
  }

  /**
   * Constructs a command that executes the given action when it initializes, and doesn't finish
   * until interrupted.
   */
  public static StartEndCommand indefiniteInstantCommand(
      Runnable onInit, Subsystem... requirements) {
    return new StartEndCommand(onInit, () -> {}, requirements);
  }

  public static <T extends Command> T withName(T command, String name) {
    return Util.returnAfterModifying(command, cmd -> cmd.setName(name));
  }
}
