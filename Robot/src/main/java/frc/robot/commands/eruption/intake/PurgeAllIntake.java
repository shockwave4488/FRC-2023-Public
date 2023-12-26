package frc.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.Intake.RollerState;
import frc.robot.subsystems.eruption.Shooter;

public class PurgeAllIntake extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public PurgeAllIntake(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(intake, indexer, shooter);
  }

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setBottomRollerState(RollerState.Off);
    intake.setTopRollerState(RollerState.ReverseMedium);
    indexer.spinBackwards();
    shooter.stop();
  }
}
