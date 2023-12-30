package frc.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class CameraWidget extends Widget {

  private final String ip;
  private final boolean autoRestart;

  /**
   * @param ip Use {##}:port for 10.44.88.##:port, 172.22.11.##:port, localhost:port
   * @param autoRestart Causes the stream to be recreated once every second in case of a disconnect
   */
  public CameraWidget(String ip, boolean autoRestart) {
    super("camera");
    this.ip = ip;
    this.autoRestart = autoRestart;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(ip);
    packer.packBoolean(autoRestart);
  }
}
