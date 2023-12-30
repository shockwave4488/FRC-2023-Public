package frc.lib.dashboard;

import frc.lib.IRobotContainer;
import frc.lib.logging.Console;
import frc.robot.Robot;
import java.util.Optional;

public interface IDashboardSupportedContainer extends IRobotContainer {

  public default Optional<DashboardServer> getDashboard() {
    Console console = Robot.getConsole();
    if (console instanceof DashboardServer dashboard) {
      return Optional.of(dashboard);
    }
    return Optional.empty();
  }

  public default void onDashboardInit(DashboardServer dashboard) {}
}
