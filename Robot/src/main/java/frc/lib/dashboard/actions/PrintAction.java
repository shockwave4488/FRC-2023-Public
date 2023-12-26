package frc.lib.dashboard.actions;

import frc.lib.dashboard.DashboardServer;
import frc.lib.logging.LeveledSmartDashboard;
import java.util.stream.Stream;

public class PrintAction implements Action {

  @Override
  public String getUsage() {
    return "<default"
        + Stream.of(LeveledSmartDashboard.values())
            .map(lvl -> " | " + lvl)
            .reduce("", (a, b) -> a + b)
        + ">";
  }

  @Override
  public void onCall(DashboardServer server, String[] args) {
    if (args.length == 0) {
      LeveledSmartDashboard manual = LeveledSmartDashboard.getManualMaxLevel();
      server.println(
          "Smart Dashboard prints: "
              + (manual == null
                  ? "default (" + LeveledSmartDashboard.getDefaultMaxLevel() + ")"
                  : manual));
      return;
    }
    if (args[0].equalsIgnoreCase("default") || args[0].equalsIgnoreCase("normal")) {
      LeveledSmartDashboard.setManualMaxLevel(null);
      server.println(
          "Prints are now the default (" + LeveledSmartDashboard.getDefaultMaxLevel() + ")");
      return;
    }
    try {
      LeveledSmartDashboard.setManualMaxLevel(LeveledSmartDashboard.valueOf(args[0].toUpperCase()));
      server.println("Forced prints to be " + LeveledSmartDashboard.getManualMaxLevel());
    } catch (IllegalArgumentException e) {
      StringBuilder options = new StringBuilder();
      for (LeveledSmartDashboard level : LeveledSmartDashboard.values()) {
        options.append("\n - ").append(level);
      }
      server.errorln("Unknown print level: " + args[0] + "\nOptions:\n - default" + options);
    }
  }
}
