package frc.lib.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class DoneCycleCommand<T extends Command> extends WrapperCommand {
  private final T command;
  private final boolean finish;
  private final List<DoneCycleMachine<?>> machines = new ArrayList<>();

  public DoneCycleCommand(T command, boolean finish) {
    super(command);
    this.command = command;
    this.finish = finish;
  }

  public DoneCycleCommand<T> withDoneCycles(DoneCycleMachine<?> machine) {
    machines.add(machine);
    return this;
  }

  public DoneCycleCommand<T> withDoneCycles(Function<T, DoneCycleMachine<?>> machineGetter) {
    return withDoneCycles(machineGetter.apply(command));
  }

  public boolean isReady() {
    return machines.stream().allMatch(machine -> machine.getStatus().isReady());
  }

  @Override
  public void execute() {
    super.execute();
    machines.forEach(machine -> machine.run());
  }

  @Override
  public boolean isFinished() {
    return finish && isReady();
  }

  @Override
  public void end(boolean interrupted) {
    machines.forEach(machine -> machine.resetDoneCycles());
  }

  public void putOnDashboard(String key) {
    SmartDashboard.putData(key, this);

    Map<String, Integer> numberOfDuplicates = new HashMap<>();
    for (int i = 0; i < machines.size(); i++) {
      DoneCycleMachine<?> machine = machines.get(i);
      String machineName = machine.getName();
      int nameDuplicates = numberOfDuplicates.getOrDefault(machineName, 0);
      SmartDashboard.putData(
          key + "/" + machineName + (nameDuplicates > 0 ? nameDuplicates + 1 : ""), machine);
      numberOfDuplicates.put(machineName, nameDuplicates + 1);
    }
  }

  public void putOnDashboard() {
    putOnDashboard(getName());
  }
}
