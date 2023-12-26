package frc.lib.dashboard.actions;

import frc.lib.dashboard.DashboardServer;

public interface Action {
  public String getUsage();

  public void onCall(DashboardServer server, String[] args);
}
