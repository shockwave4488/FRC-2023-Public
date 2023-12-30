package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.lib.commands.CommandLogger.CommandTreeNode;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 * Uncovers hidden commands for more informative RunningCommands logs.
 *
 * <p>Suppose there is a top-level command group containing a few commands. By default, none of
 * those sub-commands, as they execute, are visible to the CommandScheduler and therefore logging.
 * Only a generic name will show, which is typically insufficient for debugging the actual sequence
 * of events as they took place. Luckily, LogCommands together with the {@link CommandLogger} solve
 * this issue.
 *
 * <p>In most cases, usage of this class should be limited to the static factory methods {@link
 * #sequence(Command...) sequence()}, {@link #parallel(Command...) parallel()} {@link
 * #race(Command...) race()}, {@link #deadline(Command, Command...) deadline()}, {@link
 * #conditional(Command, Command, BooleanSupplier) conditional()}, etc. for making command
 * groups/wrappers with logged sub-commands. The main exception is when calling {@link
 * CommandGroupBase#addCommands(Command...) addCommands()} when extending a CommandGroup. In that
 * situation, the inside LogCommand function call should be {@link #arrayOf(Command, Command...)
 * arrayOf(this, [sub-commands])} in order to properly nest the commands in the outer group.
 *
 * <p>For brevity purposes, it is recommended to static import the methods of this class when
 * nesting many command groups.
 */
public class LogCommand extends WrapperCommand {
  private static final CommandLogger commandLogger = CommandLogger.getInstance();

  /**
   * Exposes the given command to the command logging mechanism. Usually there is no need to use
   * this constructor directly.
   *
   * @param command Command to log.
   * @see {@link CommandLogger}
   */
  public LogCommand(Command command) {
    super(command);
  }

  public static LogCommand[] arrayOf(Command parent, Command... commands) {
    return Stream.of(commands)
        .map(cmd -> new LogCommand(cmd).withParent(parent))
        .toArray(LogCommand[]::new);
  }

  @SuppressWarnings("removal")
  public static <T extends edu.wpi.first.wpilibj2.command.CommandGroupBase> T addTo(
      T commandGroup, Command... commands) {
    commandGroup.addCommands(arrayOf(commandGroup, commands));
    return commandGroup;
  }

  /** Factory method to create a {@link SequentialCommandGroup} with logged {@code commands}. */
  public static SequentialCommandGroup sequence(Command... commands) {
    SequentialCommandGroup group = addTo(new SequentialCommandGroup(), commands);
    group.setName("Sequence");
    return group;
  }

  /** Factory method to create a {@link ParallelCommandGroup} with logged {@code commands}. */
  public static ParallelCommandGroup parallel(Command... commands) {
    ParallelCommandGroup group = addTo(new ParallelCommandGroup(), commands);
    group.setName("Parallel");
    return group;
  }

  /** Factory method to create a {@link ParallelRaceGroup} with logged {@code commands}. */
  public static ParallelRaceGroup race(Command... commands) {
    ParallelRaceGroup group = addTo(new ParallelRaceGroup(), commands);
    group.setName("Race");
    return group;
  }

  /**
   * Factory method to create a {@link ParallelDeadlineGroup} with logged {@code deadline} and
   * {@code commands}.
   */
  public static ParallelDeadlineGroup deadline(Command deadline, Command... commands) {
    LogCommand deadlineCommand = new LogCommand(deadline);
    ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(deadlineCommand);
    deadlineGroup.setName("Deadline");
    deadlineCommand.setParent(deadlineGroup);
    return addTo(deadlineGroup, commands);
  }

  /**
   * Factory method to create a {@link ConditionalCommand} with logged {@code onTrue} and {@code
   * onFalse}.
   */
  public static ConditionalCommand either(
      Command onTrue, Command onFalse, BooleanSupplier condition) {
    LogCommand onTrueCommand = new LogCommand(onTrue);
    LogCommand onFalseCommand = new LogCommand(onFalse);
    ConditionalCommand conditionalCommand =
        new ConditionalCommand(onTrueCommand, onFalseCommand, condition);
    conditionalCommand.setName("Conditional");
    onTrueCommand.setParent(conditionalCommand);
    onFalseCommand.setParent(conditionalCommand);
    return conditionalCommand;
  }

  /**
   * Factory method to create a {@link AbortCommand} with logged {@code racedCommand} and {@code
   * followingCommand}.
   */
  public static AbortCommand abortAfter(
      Command racedCommand, Command followingCommand, Command abortDeciderCommand) {
    LogCommand racedLogCommand = new LogCommand(racedCommand);
    LogCommand followingLogCommand = new LogCommand(followingCommand);
    AbortCommand abortCommand =
        new AbortCommand(racedLogCommand, followingLogCommand, abortDeciderCommand);
    abortCommand.setName("Abort");
    racedLogCommand.setParent(abortCommand);
    followingLogCommand.setParent(abortCommand);
    return abortCommand;
  }

  public static <T extends CommandBase> T logWrappedCommand(
      Command commandToWrap,
      Function<LogCommand, T> createWrapperCommand,
      String wrapperCommandName) {
    LogCommand wrappedLogCommand = new LogCommand(commandToWrap);
    T wrapperCommand = createWrapperCommand.apply(wrappedLogCommand);
    wrapperCommand.setName(wrapperCommandName);
    wrappedLogCommand.setParent(wrapperCommand);
    return wrapperCommand;
  }

  /** Factory method to create a {@link ProxyCommand} with logged {@code command}. */
  public static ProxyCommand proxy(Command command) {
    return logWrappedCommand(command, ProxyCommand::new, "Proxy");
  }

  /** Factory method to create a {@link RepeatCommand} with logged {@code command}. */
  public static RepeatCommand repeat(Command command) {
    return logWrappedCommand(command, RepeatCommand::new, "Repeat");
  }

  /**
   * {@link Command#until(BooleanSupplier)} that preserves the original command name and is
   * compatible with logging.
   */
  public static ParallelRaceGroup endWhen(Command command, BooleanSupplier condition) {
    return logWrappedCommand(command, cmd -> cmd.until(condition), "ConditionalStop");
  }

  /**
   * {@link Command#withTimeout(double)} that preserves the original command name and is compatible
   * with logging.
   */
  public static ParallelRaceGroup endAfter(Command command, double seconds) {
    return logWrappedCommand(command, cmd -> cmd.withTimeout(seconds), "Timeout");
  }

  CommandTreeNode getTreeNode() {
    return commandLogger.getTreeNode(m_command);
  }

  public void setParent(Command parent) {
    getTreeNode().setParent(commandLogger.getTreeNode(parent));
  }

  public LogCommand withParent(Command parent) {
    setParent(parent);
    return this;
  }

  @Override
  public void execute() {
    super.execute();
    commandLogger.logCommand(this);
  }
}
