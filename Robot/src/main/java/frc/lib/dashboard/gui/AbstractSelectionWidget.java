package frc.lib.dashboard.gui;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import org.msgpack.core.MessagePacker;

/** Internally uses a SendableChooser as a backup */
public abstract class AbstractSelectionWidget<T> extends Widget {

  private final String name;
  private final String path;
  private final SendableChooser<T> chooser;

  protected AbstractSelectionWidget(String type, String name) {
    super(type);
    this.name = name;
    this.path = "/SmartDashboard/" + name;
    this.chooser = new SendableChooser<>();
    SmartDashboard.putData(name, chooser);
  }

  protected AbstractSelectionWidget(
      String type, String name, String path, SendableChooser<T> chooser) {
    super(type);
    this.name = name;
    this.path = path;
    this.chooser = chooser;
  }

  protected void addOption(String name, T value, boolean isDefault) {
    if (isDefault) {
      chooser.setDefaultOption(name, value);
    } else {
      chooser.addOption(name, value);
    }
  }

  protected T getSelected() {
    return chooser.getSelected();
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packString(path);
  }
}
