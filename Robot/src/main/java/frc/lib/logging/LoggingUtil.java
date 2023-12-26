package frc.lib.logging;

import edu.wpi.first.math.geometry.Rotation3d;

public class LoggingUtil {

  public static String formatRotation(Rotation3d rot) {
    return "(" + rot.getX() + ", " + rot.getY() + ", " + rot.getZ() + ")";
  }
}
