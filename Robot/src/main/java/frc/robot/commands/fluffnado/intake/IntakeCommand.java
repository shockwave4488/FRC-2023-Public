package frc.robot.commands.fluffnado.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.robot.subsystems.fluffnado.Intake;

public class IntakeCommand extends CommandBase {

  public static IntakeCommand in(Intake intake) {
    return new IntakeCommand(intake, Intake.Speed.FAST);
  }

  public static IntakeCommand out(Intake intake) {
    return new IntakeCommand(intake, Intake.Speed.REV_FAST);
  }

  public static IntakeCommand stop(Intake intake) {
    return new IntakeCommand(intake, Intake.Speed.STOPPED);
  }

  private final Intake intake;
  private final Intake.Speed speed;

  private static final int MIN_COUNT_CURRENT_CYCLES = 35;
  private static final int CURRENT_LIMIT = 50;

  public IntakeCommand(Intake intake, Intake.Speed speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  public DoneCycleCommand<IntakeCommand> toDoneCycleCommand() {
    return new DoneCycleCommand<>(this, true)
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(
                () -> intake.getMotorCurrent() > CURRENT_LIMIT, MIN_COUNT_CURRENT_CYCLES));
  }

  @Override
  public void execute() {
    intake.setSpeed(speed);
  }
}
