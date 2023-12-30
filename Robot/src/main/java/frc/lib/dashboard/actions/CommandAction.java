package frc.lib.dashboard.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.dashboard.DashboardServer;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class CommandAction implements Action {

  @SuppressWarnings("unchecked")
  private static Set<Command> getRunningCommands(DashboardServer server) {
    try {
      Field field = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      field.setAccessible(true);
      return (Set<Command>) field.get(CommandScheduler.getInstance());
    } catch (Exception e) {
      server.errorln("Error getting running commands", e);
      return null;
    }
  }

  private final Map<String, Command> stoppedCommands = new HashMap<>();

  @Override
  public String getUsage() {
    return "[list | (stop | resume) <name>]";
  }

  @Override
  public void onCall(DashboardServer server, String[] args) {
    switch (args.length > 0 ? args[0] : "list") {
      case "list" -> {
        Set<Command> commands = getRunningCommands(server);
        if (commands == null) {
          return;
        }
        for (Command cmd : commands) {
          StringBuilder requirements = new StringBuilder();
          if (!cmd.getRequirements().isEmpty()) {
            requirements.append(" [");
            for (Subsystem system : cmd.getRequirements()) {
              requirements.append(system.getClass().getSimpleName());
              requirements.append(", ");
            }
            requirements.delete(requirements.length() - 2, requirements.length());
            requirements.append("]");
          }
          server.println(cmd.getName() + requirements);
        }
      }
      case "stop" -> {
        if (args.length == 1) {
          server.errorln("Usage: command stop <name>");
          return;
        }
        String name = args[1];
        Set<Command> commands = getRunningCommands(server);
        if (commands == null) {
          return;
        }
        Command cmd =
            commands.stream()
                .filter(entry -> entry.getName().equalsIgnoreCase(name))
                .findFirst()
                .orElse(null);
        if (cmd == null) {
          server.errorln("Unknown command: " + name);
          return;
        }
        cmd.cancel();
        stoppedCommands.put(cmd.getName().toLowerCase(), cmd);
        server.println("Stopped command: " + cmd.getName());
      }
      case "resume" -> {
        if (args.length == 1) {
          server.errorln("Usage: command resume <name>");
          return;
        }
        String name = args[1];
        Command cmd = stoppedCommands.remove(name.toLowerCase());
        if (cmd == null) {
          server.errorln("That command wasn't previously stopped: " + name);
          return;
        }
        cmd.schedule();
        server.println("Started command: " + name);
      }
      default -> {
        server.errorln("Sub commands: list, stop, resume");
      }
    }
  }
}
