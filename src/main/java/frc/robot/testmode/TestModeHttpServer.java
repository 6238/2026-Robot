package frc.robot.testmode;

import edu.wpi.first.net.WebServer;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/** Serves the test-mode HTML report over HTTP while test mode is active. */
public class TestModeHttpServer {

  static final int PORT = 5800;
  private static final String SERVE_DIR = "/home/lvuser/test-www";
  static final String INDEX_PATH = SERVE_DIR + "/index.html";

  private TestModeHttpServer() {}

  /** Start the HTTP server and show a "test in progress" placeholder page. */
  public static void start() {
    new File(SERVE_DIR).mkdirs();
    writeFile(
        "<!DOCTYPE html><html><body style='font-family:Arial,sans-serif;margin:40px'>"
            + "<h2>Test in progress&hellip;</h2>"
            + "<p>Refresh this page when the test sequence completes.</p>"
            + "</body></html>");
    WebServer.start(PORT, SERVE_DIR);
    System.out.println(
        "[TestMode] Report server started → http://roborio-6238-frc.local:" + PORT);
  }

  /** Stop the HTTP server. Called when leaving test mode. */
  public static void stop() {
    WebServer.stop(PORT);
    System.out.println("[TestMode] Report server stopped.");
  }

  /** Replace the placeholder with the finished HTML report. */
  public static void serveReport(String html) {
    writeFile(html);
  }

  private static void writeFile(String content) {
    try (FileWriter fw = new FileWriter(INDEX_PATH)) {
      fw.write(content);
    } catch (IOException e) {
      System.err.println("[TestMode] HTTP server: could not write page: " + e.getMessage());
    }
  }
}
