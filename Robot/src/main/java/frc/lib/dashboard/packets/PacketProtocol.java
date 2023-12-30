package frc.lib.dashboard.packets;

import edu.wpi.first.networktables.NetworkTableValue;
import java.util.function.Consumer;

public interface PacketProtocol extends AutoCloseable {
  public String getName();

  public void addPacketListener(String name, Consumer<NetworkTableValue> listener);

  public void sendPacket(String name, NetworkTableValue value);

  public default void updatePackets() {}
}
