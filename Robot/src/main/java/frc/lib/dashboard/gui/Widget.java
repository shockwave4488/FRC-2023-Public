package frc.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

public abstract class Widget {

  private static long lastId = System.currentTimeMillis();

  private static long getNextId() {
    return ++lastId;
  }

  private final String type;
  private final long id;
  private boolean sizeLocked;

  public Widget(String type) {
    this.id = getNextId();
    this.type = type;
    this.sizeLocked = false;
  }

  public long getId() {
    return id;
  }

  public Widget setSizeLocked(boolean sizeLocked) {
    this.sizeLocked = sizeLocked;
    return this;
  }

  public boolean isSizeLocked() {
    return sizeLocked;
  }

  public final void sendWidget(MessagePacker packer) throws IOException {
    packer.packString(type);
    packer.packLong(id);
    packer.packBoolean(sizeLocked);
    sendData(packer);
  }

  protected abstract void sendData(MessagePacker packer) throws IOException;
}
