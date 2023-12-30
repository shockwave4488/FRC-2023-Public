package frc.lib.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.controlsystems.SupplierCache;
import java.util.function.Supplier;

public class SupplierWaitCommand extends CommandBase {
  protected Timer m_timer = new Timer();
  private final SupplierCache<Double> duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  public SupplierWaitCommand(Supplier<Double> seconds) {
    duration = new SupplierCache<>(seconds);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    duration.update();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(duration.current());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
