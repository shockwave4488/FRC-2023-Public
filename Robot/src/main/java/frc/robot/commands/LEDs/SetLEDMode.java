package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.leds.LEDMode;

public class SetLEDMode extends CommandBase {
  private final LEDController ledController;
  private final LEDMode mode;

  public SetLEDMode(LEDController ledController, LEDMode mode) {
    this.ledController = ledController;
    this.mode = mode;
    addRequirements(ledController);
  }

  @Override
  public void initialize() {
    ledController.setMode(mode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
