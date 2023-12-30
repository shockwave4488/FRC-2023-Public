package frc.lib.dashboard.gui;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.dashboard.DashboardServer;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessagePacker;

public class DashboardWebsite {

  private static final int RESPONSE_OK = 200;
  private static final int RESPONSE_PAGE_NOT_FOUND = 404;
  private static final Map<String, byte[]> files = new HashMap<>();

  private static void loadFile(String name) {
    String path = Filesystem.getDeployDirectory().getAbsolutePath() + "/friendlydashboard/" + name;
    try (var in = new FileInputStream(path)) {
      files.put(name, in.readAllBytes());
    } catch (IOException e) {
      e.printStackTrace();
      files.put(name, "<h1>File failed to load</h1>".getBytes(Charset.forName("UTF-8")));
    }
  }

  @SuppressWarnings("unused")
  private static byte[] getFile(String path, Map<String, String> replacements) {
    byte[] data = files.get(path);
    if (data == null) {
      return null;
    }
    Charset charset = Charset.forName("UTF-8");
    String output = new String(data, charset);
    for (Map.Entry<String, String> replacement : replacements.entrySet()) {
      output = output.replace("\"{@" + replacement.getKey() + "}\"", replacement.getValue());
    }
    return output.getBytes(charset);
  }

  static {
    loadFile("index.html");
    loadFile("favicon.ico");
    loadFile("fullscreen.svg");
    loadFile("debug.svg");
    loadFile("packet_protocol.svg");
    loadFile("comp_view.svg");
    loadFile("static.png");
    loadFile("folder_closed.svg");
    loadFile("folder_open.svg");
    loadFile("log.svg");

    loadFile("fields/2023.png");
    loadFile("fields/2022.png");

    loadFile("networktablesclients/nt4.js");
    loadFile("networktablesclients/msgpack/msgpack.js");
  }

  private final DashboardServer server;
  private final HttpServer webServer;
  private final GroupWidget widgets;

  public DashboardWebsite(DashboardServer server) throws IOException {
    this.server = server;

    this.webServer = HttpServer.create(new InetSocketAddress(5808), 0);
    this.webServer.createContext("/", this::handleHttpServerRequest);
    this.webServer.start();

    this.widgets = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);
  }

  private void handleHttpServerRequest(HttpExchange request) throws IOException {
    server
        .getDashboardLog()
        .println(
            LogLevel.DEBUG,
            "HttpServer "
                + request.getRequestMethod()
                + " Request: "
                + request.getRequestURI().getPath());

    String path = request.getRequestURI().getPath().substring(1);
    if (path.isEmpty()) {
      path = "index.html";
    }

    byte[] response;
    if (path.equals("logs.zip")) {
      request.sendResponseHeaders(RESPONSE_OK, 0);
      server.getLogger().generateZip(request.getResponseBody());
      request.getResponseBody().close();
      return;
    } else if (path.startsWith("logs/")) {
      Path log = LogManager.allLogsPath.resolve(path.substring("logs/".length()));
      if (server.getLogger().isLogFile(log)) { // Security check to prevent ../../<sensitive file>
        server.getLogger().flush();
        response = Files.readAllBytes(log);
      } else {
        response = null;
      }
    } else {
      response = files.get(path);
    }
    if (response == null) {
      request.sendResponseHeaders(RESPONSE_PAGE_NOT_FOUND, 0);
      request.getResponseBody().close();
      return;
    }

    String extension = path.substring(path.lastIndexOf('.') + 1);
    String contentType =
        switch (extension) {
          case "js" -> "text/javascript";
          case "svg" -> "image/svg+xml";
          case "zip" -> "application/zip";
          default -> null;
        };
    if (contentType != null) {
      request.getResponseHeaders().add("Content-Type", contentType);
    }

    request.sendResponseHeaders(RESPONSE_OK, response.length);
    OutputStream out = request.getResponseBody();
    out.write(response);
    out.close();
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public GroupWidget getWidgets() {
    return widgets;
  }

  public void sendWidgets() {
    try (ByteArrayOutputStream buf = new ByteArrayOutputStream();
        MessagePacker packer = MessagePack.newDefaultPacker(buf); ) {
      widgets.sendWidget(packer);
      packer.flush();
      DashboardServer.TABLE.putValue("Widgets", NetworkTableValue.makeRaw(buf.toByteArray()));
    } catch (IOException e) {
      // Impossible
      e.printStackTrace();
    }
  }
}
