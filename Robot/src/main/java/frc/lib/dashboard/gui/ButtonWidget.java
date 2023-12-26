package frc.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.dashboard.DashboardServer;
import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class ButtonWidget extends Widget {

  private final String name;
  private final GenericPublisher clickIdPub;
  private int clickId;

  public ButtonWidget(String name, DashboardServer dashboard, Runnable onClick) {
    super("button");
    this.name = name;
    this.clickIdPub =
        NetworkTableInstance.getDefault()
            .getTopic("/Dashboard/button_" + getId() + "/clickId")
            .genericPublish(NetworkTableType.kInteger.getValueStr());
    this.clickIdPub.setInteger(clickId);
    SmartDashboard.putData(
        name,
        new InstantCommand(
                () -> {
                  // This notifies the dashboard that the click was received (sometimes it isn't)
                  clickIdPub.setInteger(++clickId);
                  onClick.run();
                })
            .ignoringDisable(true)
            .withName(name));
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
  }
}
