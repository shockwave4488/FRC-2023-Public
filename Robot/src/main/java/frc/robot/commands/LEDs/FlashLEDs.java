package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.leds.LEDMode;

public class FlashLEDs extends CommandBase {
  private final LEDController ledController;
  private final LEDMode flashMode;
  private final double duration;
  private LEDMode previousMode;
  private double startTime;

  public FlashLEDs(LEDController ledController, LEDMode mode, double durationSeconds) {
    this.ledController = ledController;
    this.flashMode = mode;
    this.duration = durationSeconds;
    addRequirements(ledController);
  }

  @Override
  public void initialize() {
    previousMode = ledController.getMode();
    startTime = Timer.getFPGATimestamp();
    ledController.setMode(flashMode);
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > duration;
  }

  @Override
  public void end(boolean interrupted) {
    ledController.setMode(previousMode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
