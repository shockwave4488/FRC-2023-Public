package frc.lib.dashboard.packets;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.msgpack.core.MessageBufferPacker;
import org.msgpack.core.MessageFormat;
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessageUnpacker;

public class WebSocketPacketProtocol extends WebSocketServer implements PacketProtocol {

  private static byte[] serialize(String name, NetworkTableValue value) {
    try (MessageBufferPacker data = MessagePack.newDefaultBufferPacker()) {
      data.packString(name);
      switch (value.getType()) {
        case kBoolean -> data.packBoolean(value.getBoolean());
        case kBooleanArray -> {
          data.packArrayHeader(value.getBooleanArray().length + 1);
          data.packByte((byte) value.getType().ordinal());
          for (boolean entry : value.getBooleanArray()) {
            data.packBoolean(entry);
          }
        }
        case kDouble -> data.packDouble(value.getDouble());
        case kDoubleArray -> {
          data.packArrayHeader(value.getDoubleArray().length + 1);
          data.packByte((byte) value.getType().ordinal());
          for (double entry : value.getDoubleArray()) {
            data.packDouble(entry);
          }
        }
        case kFloat -> data.packFloat(value.getFloat());
        case kFloatArray -> {
          data.packArrayHeader(value.getFloatArray().length + 1);
          data.packByte((byte) value.getType().ordinal());
          for (float entry : value.getFloatArray()) {
            data.packFloat(entry);
          }
        }
        case kInteger -> data.packLong(value.getInteger());
        case kIntegerArray -> {
          data.packArrayHeader(value.getIntegerArray().length + 1);
          data.packByte((byte) value.getType().ordinal());
          for (long entry : value.getIntegerArray()) {
            data.packLong(entry);
          }
        }
        case kRaw -> {
          data.packBinaryHeader(value.getRaw().length);
          data.writePayload(value.getRaw());
        }
        case kString -> data.packString(value.getString());
        case kStringArray -> {
          data.packArrayHeader(value.getStringArray().length + 1);
          data.packByte((byte) value.getType().ordinal());
          for (String entry : value.getStringArray()) {
            data.packString(entry);
          }
        }
        case kUnassigned -> {}
      }
      return data.toByteArray();
    } catch (IOException e) {
      // Impossible
      throw new UncheckedIOException(e);
    }
  }

  private static NetworkTableValue deserialize(MessageUnpacker data) throws IOException {
    return switch (data.getNextFormat().getValueType()) {
      case ARRAY -> {
        NetworkTableValue[] array = new NetworkTableValue[data.unpackArrayHeader() - 1];
        NetworkTableType type = NetworkTableType.values()[data.unpackByte()];
        for (int i = 0; i < array.length; i++) {
          array[i] = deserialize(data);
        }
        yield switch (type) {
          case kBooleanArray -> NetworkTableValue.makeBooleanArray(
              Stream.of(array).map(NetworkTableValue::getBoolean).toArray(Boolean[]::new));
          case kDoubleArray -> NetworkTableValue.makeDoubleArray(
              Stream.of(array).map(NetworkTableValue::getDouble).toArray(Double[]::new));
          case kFloatArray -> NetworkTableValue.makeFloatArray(
              Stream.of(array).map(NetworkTableValue::getFloat).toArray(Float[]::new));
          case kIntegerArray -> NetworkTableValue.makeIntegerArray(
              Stream.of(array).map(NetworkTableValue::getInteger).toArray(Long[]::new));
          case kStringArray -> NetworkTableValue.makeStringArray(
              Stream.of(array).map(NetworkTableValue::getString).toArray(String[]::new));
          default -> throw new IOException("Not an array: " + type.getValueStr());
        };
      }
      case BINARY -> NetworkTableValue.makeRaw(data.readPayload(data.unpackBinaryHeader()));
      case BOOLEAN -> NetworkTableValue.makeBoolean(data.unpackBoolean());
      case FLOAT -> data.getNextFormat() == MessageFormat.FLOAT32
          ? NetworkTableValue.makeFloat(data.unpackFloat())
          : NetworkTableValue.makeDouble(data.unpackDouble());
      case INTEGER -> NetworkTableValue.makeInteger(data.unpackLong());
      case STRING -> NetworkTableValue.makeString(data.unpackString());
      default -> throw new IOException("Unsupported type: " + data.getNextFormat().getValueType());
    };
  }

  private static int findPort(int... ports) throws IOException {
    for (int port : ports) {
      try (ServerSocket server = new ServerSocket(port)) {
        server.setReuseAddress(true);
        return port;
      } catch (IOException e) {
        // Expected, try next port
      }
    }
    throw new IOException("Unable to find an open port!");
  }

  private final LogManager logger;
  private final Map<String, List<Consumer<NetworkTableValue>>> listeners;

  public WebSocketPacketProtocol(LogManager logger, int... ports) throws IOException {
    super(new InetSocketAddress(findPort(ports)));
    this.logger = logger;
    this.listeners = new HashMap<>();
    this.start();
  }

  @Override
  public String getName() {
    return PacketProtocolFactory.WEBSOCKET_PROTOCOL;
  }

  @Override
  public void addPacketListener(String name, Consumer<NetworkTableValue> listener) {
    listeners.computeIfAbsent(name, key -> new ArrayList<>()).add(listener);
  }

  @Override
  public void sendPacket(String name, NetworkTableValue value) {
    try {
      byte[] data = serialize(name, value);
      for (WebSocket conn : getConnections()) {
        conn.send(data);
      }
    } catch (Exception e) {
      onError(null, e);
    }
  }

  private void receivePacket(String name, NetworkTableValue value) {
    try {
      List<Consumer<NetworkTableValue>> specificListeners = listeners.get(name);
      if (specificListeners == null) {
        return;
      }
      for (Consumer<NetworkTableValue> listener : specificListeners) {
        listener.accept(value);
      }
    } catch (Exception e) {
      onError(null, e);
    }
  }

  @Override
  public void onStart() {
    logger
        .getLogFile("Dashboard")
        .println(LogLevel.INFO, "Dashboard web socket started on port " + getPort());
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {}

  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {}

  @Override
  public void onMessage(WebSocket conn, String message) {
    try {
      receivePacket("", NetworkTableValue.makeString(message));
    } catch (Exception e) {
      onError(conn, e);
    }
  }

  @Override
  public void onMessage(WebSocket conn, ByteBuffer message) {
    try (MessageUnpacker in = MessagePack.newDefaultUnpacker(message)) {
      receivePacket(in.unpackString(), deserialize(in));
    } catch (Exception e) {
      onError(conn, e);
    }
  }

  @Override
  public void onError(WebSocket conn, Exception e) {
    logger.getLogFile("Dashboard").println(LogLevel.ERROR, e);
  }

  @Override
  public void close() throws Exception {
    this.stop();
  }
}
