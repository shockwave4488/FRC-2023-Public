package frc.lib.dashboard.gui;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.msgpack.core.MessagePacker;

public class GroupWidget extends Widget {

  public enum GroupWidgetDirection {
    HORIZONTAL,
    VERTICAL
  }

  private final List<Widget> widgets;
  private GroupWidgetDirection dir;
  private boolean stretched;
  private boolean bordered;

  /**
   * Create a visual list of widgets
   *
   * @param dir The direction of the list
   * @param stretched If the list items should be stretched to fill the entire widget
   * @param bordered If the list items should have a border in between them
   */
  public GroupWidget(GroupWidgetDirection dir, boolean stretched, boolean bordered) {
    super("group");
    this.widgets = new ArrayList<>();
    this.dir = dir;
    this.stretched = stretched;
    this.bordered = bordered;
  }

  public void addWidget(Widget widget) {
    this.widgets.add(widget);
  }

  public void setDirection(GroupWidgetDirection dir) {
    this.dir = dir;
  }

  public GroupWidgetDirection getDirection() {
    return dir;
  }

  public void setStretched(boolean stretched) {
    this.stretched = stretched;
  }

  public boolean isStretched() {
    return stretched;
  }

  public void setBordered(boolean bordered) {
    this.bordered = bordered;
  }

  public boolean isBordered() {
    return this.bordered;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packBoolean(dir == GroupWidgetDirection.VERTICAL);
    packer.packBoolean(stretched);
    packer.packBoolean(bordered);
    packer.packInt(widgets.size());
    for (Widget widget : widgets) {
      widget.sendWidget(packer);
    }
  }
}
