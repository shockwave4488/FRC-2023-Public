package frc.lib.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Collections;

public class AbortCommand extends CommandBase {
  private final Command racedCommand;
  private final Command followingCommand;
  private final Command abortDeciderCommand;
  private boolean racedCommandFinished;
  private boolean aborted;

  /**
   * Races {@code racedCommand} with {@code abortDeciderCommand}. If {@code racedCommand} finishes
   * first, then the command moves on to {@code followingCommand}. Else, the command ends. If {@code
   * racedCommand} ends in the same tick as {@code abortDeciderCommand}, then the code continues as
   * if {@code racedCommand} ended first
   *
   * <p>This differs from a simple {@link edu.wpi.first.wpilibj2.command.ParallelRaceGroup
   * ParallelRaceGroup} in that the {@code abortDeciderCommand} has the power to prevent the next
   * command from starting if it ends first. If {@code racedCommand} ends first, {@code
   * abortDeciderCommand} has no effect on the {@code followingCommand} that runs subsequently.
   */
  public AbortCommand(Command racedCommand, Command followingCommand, Command abortDeciderCommand) {
    this.racedCommand = requireNonNullParam(racedCommand, "racedCommand", "AbortCommand");
    this.followingCommand =
        requireNonNullParam(followingCommand, "followingCommand", "AbortCommand");
    this.abortDeciderCommand =
        requireNonNullParam(abortDeciderCommand, "abortDeciderCommand", "AbortCommand");

    if (!Collections.disjoint(
        racedCommand.getRequirements(), abortDeciderCommand.getRequirements())) {
      throw new IllegalArgumentException(
          String.format(
              """
          Multiple commands in a parallel composition cannot require the same subsystems! \
           'racedCommand' (%s) and 'abortDeciderCommand' (%s) share requirements.""",
              racedCommand.getName(), abortDeciderCommand.getName()));
    }

    CommandScheduler.getInstance()
        .registerComposedCommands(racedCommand, followingCommand, abortDeciderCommand);

    m_requirements.addAll(racedCommand.getRequirements());
    m_requirements.addAll(followingCommand.getRequirements());
    m_requirements.addAll(abortDeciderCommand.getRequirements());
  }

  @Override
  public void initialize() {
    racedCommandFinished = false;
    aborted = false;
    racedCommand.initialize();
    abortDeciderCommand.initialize();
  }

  @Override
  public void execute() {
    if (racedCommandFinished) {
      followingCommand.execute();
    } else {
      racedCommand.execute();
      if (racedCommand.isFinished()) {
        racedCommand.end(false);
        abortDeciderCommand.end(!abortDeciderCommand.isFinished());
        racedCommandFinished = true;
        followingCommand.initialize();
      } else {
        abortDeciderCommand.execute();
        if (abortDeciderCommand.isFinished()) {
          abortDeciderCommand.end(false);
          racedCommand.end(true);
          aborted = true;
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (!racedCommandFinished && aborted)
        || (racedCommandFinished && followingCommand.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    if (racedCommandFinished) {
      followingCommand.end(interrupted);
    } else {
      racedCommand.end(interrupted);
      abortDeciderCommand.end(interrupted);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return abortDeciderCommand.runsWhenDisabled()
        && racedCommand.runsWhenDisabled()
        && followingCommand.runsWhenDisabled();
  }
}
