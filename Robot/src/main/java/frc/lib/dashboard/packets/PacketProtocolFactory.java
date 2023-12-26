package frc.lib.dashboard.packets;

import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PacketProtocolFactory {

  public interface PacketProtocolSupplier {
    public PacketProtocol get() throws IOException;
  }

  // These must match index.html for protocols supported by the dashboard
  public static final String NETWORK_TABLE_PROTOCOL = "NetworkTable";
  public static final String WEBSOCKET_PROTOCOL = "WebSocket";

  private static final Map<String, PacketProtocolSupplier> registeredProtocols = new HashMap<>();

  public static void registerProtocol(String name, PacketProtocolSupplier protocol) {
    registeredProtocols.put(name, protocol);
  }

  public static void registerDefaultProtocols(LogManager logger) {
    registerProtocol(NETWORK_TABLE_PROTOCOL, () -> new NetworkTablePacketProtocol(logger));
    registerProtocol(
        WEBSOCKET_PROTOCOL, () -> new WebSocketPacketProtocol(logger, 5809, 5807, 5806));
  }

  public static PacketProtocol create(String name) throws IOException {
    PacketProtocolSupplier supplier = registeredProtocols.get(name);
    if (supplier == null) {
      if (registeredProtocols.isEmpty()) {
        throw new IllegalArgumentException(
            "There are no registered packet protocols! Has registerDefaultProtocols been called? ("
                + name
                + ")");
      } else {
        throw new IllegalArgumentException("Unknown packet protocol: " + name);
      }
    }
    PacketProtocol protocol = supplier.get();
    if (!name.equals(protocol.getName())) {
      throw new IllegalStateException(
          "Created packet protocol has a mismatched name: " + name + " != " + protocol.getName());
    }
    return protocol;
  }

  public static PacketProtocol createOrFallback(
      String name, LogManager logger, Supplier<PacketProtocol> fallback) {
    try {
      return create(name);
    } catch (Exception e) {
      logger
          .getLogFile("Dashboard")
          .println(LogLevel.ERROR, "Error while creating packet protocol '" + name + "'", e);
      return fallback.get();
    }
  }

  public static PacketProtocol createOrFallback(String name, LogManager logger) {
    return createOrFallback(name, logger, () -> new NetworkTablePacketProtocol(logger));
  }
}
