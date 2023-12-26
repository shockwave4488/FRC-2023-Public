package frc.lib.commands;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.LogManager;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public final class CommandLogger implements Sendable, AutoCloseable {
  static class CommandTreeNode {
    private final Supplier<String> name;
    private final List<CommandTreeNode> children = new ArrayList<>();
    private Optional<CommandTreeNode> parent = Optional.empty();
    private boolean excluded = false;

    public CommandTreeNode(Supplier<String> commandName) {
      name = commandName;
    }

    public void addChild(CommandTreeNode commandNode) {
      children.add(commandNode);
    }

    public void setParent(CommandTreeNode commandNode) {
      commandNode.addChild(this);
      parent = Optional.of(commandNode);
    }

    public boolean hasAncestor(CommandTreeNode commandNode) {
      return getParent()
          .map(cmd -> (cmd != commandNode) ? cmd.hasAncestor(commandNode) : true)
          .orElse(this == commandNode);
    }

    public Optional<CommandTreeNode> getParent() {
      return parent;
    }

    public List<CommandTreeNode> getChildren() {
      return children;
    }

    public boolean isRoot() {
      return parent.isEmpty();
    }

    public boolean isLeaf() {
      return children.isEmpty();
    }

    public boolean isExcluded() {
      return excluded;
    }

    public String getName() {
      return parent.map(cmd -> cmd.getName() + " -> ").orElse("") + name.get();
    }

    public void exclude() {
      excluded = true;
      children.forEach(CommandTreeNode::exclude);
    }
  }

  private static CommandLogger instance;

  /** Returns the CommandLogger singleton. */
  @SuppressFBWarnings("MS_EXPOSE_REP")
  public static synchronized CommandLogger getInstance() {
    if (instance == null) {
      instance = new CommandLogger();
    }
    return instance;
  }

  private boolean disabled;
  private final Map<Command, CommandTreeNode> commandNodes = new HashMap<>();
  private final Set<CommandTreeNode> runningLeafCommands = new HashSet<>();
  private String[] loggedNames = new String[0];

  private CommandLogger() {
    SendableRegistry.addLW(this, "CommandLogger");

    CommandScheduler.getInstance().onCommandExecute(this::logCommand);
  }

  public void setLoggerTrackable(LogManager logger) {
    logger
        .getLogFile("CommandLogger")
        .addTracker("Running", () -> String.join(",", loggedNames), 4);
  }

  public void excludeCommand(Command command) {
    getTreeNode(command).exclude();
  }

  /**
   * Adds a {@link LogCommand} to the current set of running commands. Since the running commands
   * are periodically refreshed, this method must be called every cycle while the command is
   * running.
   */
  void logCommand(LogCommand command) {
    if (!disabled) {
      logCommand(command.getTreeNode());
    }
  }

  private void logCommand(Command command) {
    if (!disabled) {
      // Handle the case a LogCommand is seen by the scheduler, such as due to a ProxyCommand.
      if (!(command instanceof LogCommand)) {
        logCommand(getTreeNode(command));
      }
    }
  }

  private void logCommand(CommandTreeNode command) {
    if (command.isLeaf()) {
      Optional<CommandTreeNode> curAncestor = Optional.of(command);
      do {
        if (!curAncestor.get().isExcluded()) {
          runningLeafCommands.add(curAncestor.get());
          break;
        }
        curAncestor = curAncestor.get().getParent();
      } while (curAncestor.isPresent());
    }
  }

  CommandTreeNode getTreeNode(Command command) {
    CommandTreeNode commandNode = commandNodes.get(command);
    if (commandNode == null) {
      commandNode = new CommandTreeNode(command::getName);
      commandNodes.put(command, commandNode);
    }
    return commandNode;
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Constructs a list of names of the logged commands and clears the previous iteration of running
   * commands. Intended to be called after the CommandScheduler runs in {@link
   * frc.robot.Robot#robotPeriodic() robotPeriodic()}.
   */
  public void update() {
    if (!disabled) {
      loggedNames =
          runningLeafCommands.stream().map(CommandTreeNode::getName).toArray(String[]::new);
      runningLeafCommands.clear();
    }
  }

  /** Disables the command logger. */
  public void disable() {
    disabled = true;
  }

  /** Enables the command logger. */
  public void enable() {
    disabled = false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty(
        "Enabled",
        () -> !disabled,
        enable -> {
          if (enable) enable();
          else disable();
        });
    builder.addStringArrayProperty("Names", () -> loggedNames, null);
  }
}
