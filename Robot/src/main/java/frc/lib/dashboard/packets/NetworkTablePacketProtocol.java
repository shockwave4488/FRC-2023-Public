package frc.lib.dashboard.packets;

import edu.wpi.first.networktables.NetworkTableValue;
import frc.lib.logging.LogFile;
import frc.lib.logging.LogManager;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class NetworkTablePacketProtocol implements PacketProtocol {

  private final LogFile dashboardLog;
  private final Map<String, List<Consumer<NetworkTableValue>>> listeners;

  public NetworkTablePacketProtocol(LogManager logger) {
    this.dashboardLog = logger.getLogFile("Dashboard");
    this.listeners = new HashMap<>();
    NetworkTablePackets.init();
  }

  @Override
  public String getName() {
    return PacketProtocolFactory.NETWORK_TABLE_PROTOCOL;
  }

  @Override
  public void addPacketListener(String name, Consumer<NetworkTableValue> listener) {
    listeners.computeIfAbsent(name, key -> new ArrayList<>()).add(listener);
  }

  @Override
  public void sendPacket(String name, NetworkTableValue value) {
    NetworkTablePackets.sendPacket(name, value);
  }

  @Override
  public void updatePackets() {
    for (Map.Entry<String, List<Consumer<NetworkTableValue>>> specificListeners :
        listeners.entrySet()) {
      List<NetworkTableValue> received =
          NetworkTablePackets.readPacket(specificListeners.getKey(), dashboardLog);
      for (NetworkTableValue packet : received) {
        for (Consumer<NetworkTableValue> listener : specificListeners.getValue()) {
          listener.accept(packet);
        }
      }
    }
  }

  @Override
  public void close() throws Exception {}
}
