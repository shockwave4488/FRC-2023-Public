package frc.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class DisplayWidget extends Widget {

  private final String value;

  /**
   * Display a string
   *
   * @param value The value to display, using {key} to reference a network table entry
   */
  public DisplayWidget(String value) {
    super("display");
    this.value = value;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(value);
  }
}
