package frc.lib.operator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A trigger for the d-pad in any variation of some direction. For example, deg = 0 will select up
 * left, up, and up right.
 */
public class POVRange extends Trigger {
  public static final int UP = 0;
  public static final int RIGHT = 90;
  public static final int DOWN = 180;
  public static final int LEFT = 270;

  public POVRange(XboxController controller, int deg) {
    super(() -> Math.abs(controller.getPOV() - deg) <= 45);
  }

  public POVRange(CommandXboxController controller, int deg) {
    super(controller.pov(deg).or(controller.pov(deg - 45)).or(controller.pov(deg + 45)));
  }
}
