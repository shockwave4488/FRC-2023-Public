package frc.lib.dashboard.gui;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class DropdownWidget<T> extends AbstractSelectionWidget<T> {

  public DropdownWidget(String name) {
    super("dropdown", name);
  }

  public DropdownWidget(String name, String path, SendableChooser<T> chooser) {
    super("dropdown", name, path, chooser);
  }

  public void addOption(String name, T value) {
    super.addOption(name, value, false);
  }

  @Override
  public T getSelected() { // Increase visibility
    return super.getSelected();
  }
}
