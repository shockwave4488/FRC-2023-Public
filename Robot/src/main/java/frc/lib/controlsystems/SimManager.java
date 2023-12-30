package frc.lib.controlsystems;

import com.revrobotics.REVPhysicsSim;

public class SimManager {

  private static long lastSimPeriodic;
  private static long deltaSimTime;

  public static long getDeltaSimTime() {
    return deltaSimTime;
  }

  public static void simulationInit() {
    lastSimPeriodic = System.currentTimeMillis();
  }

  public static void simulationPeriodic() {
    long time = System.currentTimeMillis();
    deltaSimTime = time - lastSimPeriodic;
    lastSimPeriodic = time;

    REVPhysicsSim.getInstance().run();
  }
}
