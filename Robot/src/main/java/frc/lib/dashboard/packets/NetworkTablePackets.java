package frc.lib.dashboard.packets;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.Topic;
import frc.lib.dashboard.DashboardServer;
import frc.lib.logging.LogFile;
import frc.lib.logging.LogLevel;
import frc.lib.misc.RateLimitGroup;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 *
 *
 * <pre>
 * Sends a "packet" over Network Tables for one-time messages.
 * This allows packets to be queued up, so new packets don't override old ones that haven't been read yet.
 * Internally, this uses a table named "Dashboard", which uses a specific entry format:
 * long PacketName@ = The id of the latest packet
 * any PacketName@1234 = The packet with the id 1234
 * After timeout (default 5s) is passed, PacketName@1234 will be removed to prevent memory issues
 * At that point, the packet will be lost by any receiver that hasn't handled it yet
 * </pre>
 */
class NetworkTablePackets {
  private static final Map<String, Long> ids = new HashMap<>();
  private static final Map<String, GenericPublisher> idPublishers = new HashMap<>();
  private static final Set<String> updatedPackets = new HashSet<>();
  private static final RateLimitGroup<String> rateLimit =
      new RateLimitGroup<>(15, 1000, RateLimitGroup.Distributor.makePercent(0.75));
  private static final int maxPackets = 50;
  private static volatile int currentPackets = 0;

  static {
    DashboardServer.TABLE.addListener(
        EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
        (table, key, event) -> {
          int i = key.indexOf('@');
          if (i != -1) {
            updatedPackets.add(key.substring(0, i));
          }
        });
  }

  public static void init() {
    // Load class
  }

  private static synchronized void checkName(String name) throws IllegalArgumentException {
    if (name.contains("@") || name.contains("" + NetworkTable.PATH_SEPARATOR)) {
      throw new IllegalArgumentException(
          "Packet names cannot include the character '@' or '" + NetworkTable.PATH_SEPARATOR + "'");
    }
  }

  /**
   * Read all the packets sent since that last call of this method
   *
   * @param name The name of the packet
   * @param log A log file for logging errors
   * @return All of the packets
   * @throws IllegalArgumentException If the packet name is invalid
   */
  public static synchronized List<NetworkTableValue> readPacket(String name, LogFile log)
      throws IllegalArgumentException {
    checkName(name);

    List<NetworkTableValue> output = new ArrayList<>();

    if (!updatedPackets.contains(name)) {
      return output;
    }

    NetworkTableValue id = DashboardServer.TABLE.getValue(name + "@");
    if (id == null || !id.isValid()) {
      return output;
    }
    if (!id.isInteger()) {
      log.println(LogLevel.ERROR, "Invalid packet id format: " + id.getType().getValueStr());
      return output;
    }
    long expectedId = ids.getOrDefault(name, 0L);
    if (id.getInteger() < expectedId) {
      return output;
    }
    long numPackets = id.getInteger() - expectedId + 1;

    long lastReadId = -1L;
    List<Long> lostPackets = new ArrayList<>();
    for (long i = 0; i < numPackets; i++) {
      NetworkTableValue packet = DashboardServer.TABLE.getValue(name + "@" + (expectedId + i));
      if (packet != null && packet.isValid()) {
        output.add(packet);
        lastReadId = expectedId + i;
      } else {
        lostPackets.add(expectedId + i);
      }
    }
    if (lastReadId != -1L) {
      ids.put(name, lastReadId + 1);
      for (Long lostPacket : lostPackets) {
        if (lostPacket < lastReadId) {
          // This doesn't always indicate a problem; it can also be caused by the
          // robot restarting without refreshing the dashboard (which is supported)
          log.println(LogLevel.DEBUG, "Packet " + name + "@" + lostPacket + " was lost!");
        }
      }
    }

    if (lostPackets.isEmpty() || lostPackets.get(lostPackets.size() - 1) < lastReadId) {
      updatedPackets.remove(name);
    }

    return output;
  }

  /**
   * Send a packet over NetworkTables
   *
   * @param name The name of the packet
   * @param value The packet payload
   * @param timeout The time the packet should remain on NetworkTables
   * @return If the packet could be sent (false if blocked by rate limit)
   * @throws IllegalArgumentException If the packet name is invalid
   */
  public static synchronized boolean sendPacket(String name, NetworkTableValue value, int timeout)
      throws IllegalArgumentException {
    checkName(name);
    if (!rateLimit.exec(name) || currentPackets >= maxPackets) {
      return false;
    }

    long id =
        ids.compute(
            name,
            (key, prevId) -> {
              if (prevId == null) {
                NetworkTableValue restoredId = DashboardServer.TABLE.getValue(name + "@");
                if (restoredId == null || !restoredId.isValid() || !restoredId.isInteger()) {
                  return 0L;
                }
                return restoredId.getInteger() + 1;
              } else {
                return prevId + 1;
              }
            });
    GenericPublisher publisher =
        DashboardServer.TABLE
            .getTopic(name + "@" + id)
            .genericPublish(value.getType().getValueStr());
    publisher.set(value);
    currentPackets++;

    GenericPublisher idPublisher = idPublishers.get(name);
    if (idPublisher == null) {
      Topic topic = DashboardServer.TABLE.getTopic(name + "@");
      idPublisher = topic.genericPublish(NetworkTableType.kInteger.getValueStr());
      topic.setRetained(true);
    }
    idPublisher.setInteger(id);

    // Automatically removes the packet after timeout
    Thread thread =
        new Thread(
            () -> {
              try {
                Thread.sleep(timeout);
              } catch (InterruptedException e) {
              }
              publisher.close();
              currentPackets--;
            },
            "DashboardPackets.sendPacket(name = " + name + ", timeout = " + timeout + ")");
    thread.setDaemon(true);
    thread.start();

    return true;
  }

  /**
   * Send a packet over NetworkTables, with a 5s timeout
   *
   * @param name The name of the packet
   * @param value The packet payload
   * @return If the packet could be sent (false if blocked by rate limit)
   * @throws IllegalArgumentException If the packet name is invalid
   */
  public static synchronized boolean sendPacket(String name, NetworkTableValue value)
      throws IllegalArgumentException {
    return sendPacket(name, value, 5000);
  }
}
