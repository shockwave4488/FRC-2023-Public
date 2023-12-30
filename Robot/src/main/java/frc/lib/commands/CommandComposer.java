package frc.lib.commands;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class CommandComposer<T extends Command> extends CommandBase {
  protected T composedCommand;
  protected Set<Subsystem> curRequirements;

  public CommandComposer(Subsystem... requirements) {
    addRequirements(requirements);
    curRequirements = Set.of(requirements);
  }

  /** Set composed command. Static requirements should be added before calling this method. */
  protected void setComposedCommand(T command) {
    composedCommand = command;
    Set<Subsystem> requirements = new HashSet<>(command.getRequirements());
    requirements.addAll(m_requirements);
    curRequirements = Collections.unmodifiableSet(requirements);
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public T getComposedCommand() {
    return composedCommand;
  }

  @Override
  @SuppressFBWarnings(
      value = "EI_EXPOSE_REP",
      justification = "curRequirements is already unmodifiable")
  public Set<Subsystem> getRequirements() {
    return curRequirements;
  }

  @Override
  public void initialize() {
    if (composedCommand != null) {
      composedCommand.initialize();
    }
  }

  @Override
  public void execute() {
    if (composedCommand != null) {
      composedCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return (composedCommand != null) ? composedCommand.isFinished() : false;
  }

  @Override
  public void end(boolean interrupted) {
    if (composedCommand != null) {
      composedCommand.end(interrupted);
    }
  }
}
