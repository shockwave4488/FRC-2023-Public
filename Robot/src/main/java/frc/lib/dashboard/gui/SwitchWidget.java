package frc.lib.dashboard.gui;

public class SwitchWidget extends AbstractSelectionWidget<Boolean> {

  private final boolean defaultValue;

  public SwitchWidget(String name, boolean enabled) {
    super("switch", name);
    this.defaultValue = enabled;

    addOption("false", false, !enabled);
    addOption("true", true, enabled);
  }

  public boolean isEnabled() {
    Boolean value = getSelected();
    if (value == null) {
      return defaultValue;
    }
    return value;
  }
}
