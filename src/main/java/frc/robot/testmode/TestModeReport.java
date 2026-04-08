package frc.robot.testmode;

import frc.robot.testmode.TestModeRunner.MotorResult;
import frc.robot.testmode.TestModeRunner.TestStatus;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Generates and writes a self-contained HTML motor-test report. */
public class TestModeReport {

  /**
   * Builds a self-contained HTML string from the completed motor test results.
   *
   * @param results list of per-motor results from {@link TestModeRunner}
   * @param robotSerial roboRIO serial number
   * @param gitSha full git SHA (will be truncated to 8 chars in display)
   */
  public static String generate(
      List<MotorResult> results, String robotSerial, String gitSha, boolean baselineMode) {
    String date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"));

    long passCount = results.stream().filter(r -> r.status() == TestStatus.PASS).count();
    long warnCount = results.stream().filter(r -> r.status() == TestStatus.WARN).count();
    long failCount =
        results.stream()
            .filter(
                r ->
                    r.status() == TestStatus.FAIL
                        || r.status() == TestStatus.WARN
                        || r.status() == TestStatus.NO_BASELINE)
            .count();

    String overallLabel;
    String overallColor;
    if (baselineMode) {
      overallLabel = "BASELINE RECORDED";
      overallColor = "#1565c0";
    } else if (failCount > 0) {
      overallLabel = "FAIL";
      overallColor = "#c62828";
    } else {
      overallLabel = "PASS";
      overallColor = "#2e7d32";
    }

    String sha8 = gitSha.length() >= 8 ? gitSha.substring(0, 8) : gitSha;

    // Build table rows: failed/warned first, then passing, each grouped by step
    boolean anyFail = results.stream().anyMatch(r -> r.status() == TestStatus.FAIL);
    boolean anyWarn = results.stream().anyMatch(r -> r.status() == TestStatus.WARN);

    // Partition: non-passing rows first, then passing rows
    List<MotorResult> nonPassing =
        results.stream()
            .filter(r -> r.status() != TestStatus.PASS && r.status() != TestStatus.NO_BASELINE)
            .toList();
    List<MotorResult> passing =
        results.stream()
            .filter(r -> r.status() == TestStatus.PASS || r.status() == TestStatus.NO_BASELINE)
            .toList();

    StringBuilder rows = new StringBuilder();

    // Render a section of results grouped by step
    java.util.function.Consumer<List<MotorResult>> renderSection =
        sectionResults -> {
          List<String> sectionSteps =
              sectionResults.stream().map(MotorResult::step).distinct().toList();
          for (String step : sectionSteps) {
            List<MotorResult> stepResults =
                sectionResults.stream().filter(r -> r.step().equals(step)).toList();
            rows.append("<tr class='sh'><td colspan='7'>")
                .append(escHtml(step))
                .append("</td></tr>\n");
            for (MotorResult r : stepResults) {
              String statusColor = statusColor(r.status());
              String deltaStr;
              String baselineStr;
              if (Double.isNaN(r.baselineMean()) || r.baselineMean() <= 0) {
                deltaStr = "&mdash;";
                baselineStr = "&mdash;";
              } else {
                double pct = (r.measuredMean() / r.baselineMean() - 1.0) * 100.0;
                deltaStr = String.format("%+.0f%%", pct);
                baselineStr = String.format("%.3f", r.baselineMean());
              }
              double scale =
                  Math.max(
                      r.measuredMean(),
                      Double.isNaN(r.baselineMean()) ? r.measuredMean() : r.baselineMean());
              int mBar = scale > 0 ? (int) Math.round(r.measuredMean() / scale * 110) : 0;
              int bBar =
                  (scale > 0 && !Double.isNaN(r.baselineMean()))
                      ? (int) Math.round(r.baselineMean() / scale * 110)
                      : 0;
              String svg =
                  "<svg width='120' height='22' aria-hidden='true'>"
                      + "<rect x='0' y='1'  width='"
                      + bBar
                      + "' height='9' fill='#90a4ae' rx='2' title='baseline'/>"
                      + "<rect x='0' y='12' width='"
                      + mBar
                      + "' height='9' fill='"
                      + statusColor
                      + "' rx='2' title='measured'/>"
                      + "</svg>";
              rows.append("<tr>")
                  .append("<td>")
                  .append(escHtml(r.motorKey()))
                  .append("</td>")
                  .append("<td>")
                  .append(baselineStr)
                  .append("</td>")
                  .append("<td>")
                  .append(String.format("%.3f", r.measuredMean()))
                  .append("</td>")
                  .append("<td>")
                  .append(String.format("%.3f", r.measuredMin()))
                  .append("</td>")
                  .append("<td>")
                  .append(deltaStr)
                  .append("</td>")
                  .append("<td>")
                  .append(svg)
                  .append("</td>")
                  .append("<td style='color:")
                  .append(statusColor)
                  .append(";font-weight:700'>")
                  .append(r.status())
                  .append("</td>")
                  .append("</tr>\n");
            }
          }
        };

    if (!nonPassing.isEmpty()) {
      rows.append(
          "<tr class='sh'><td colspan='7' style='background:#ffebee;color:#b71c1c'>"
              + "&#9888; FAILED / WARNED MOTORS</td></tr>\n");
      renderSection.accept(nonPassing);
      if (!passing.isEmpty()) {
        rows.append(
            "<tr class='sh'><td colspan='7' style='background:#e8f5e9;color:#1b5e20'>"
                + "&#10003; PASSING MOTORS</td></tr>\n");
      }
    }
    renderSection.accept(passing);

    if (baselineMode) {
      Logger.recordOutput("TestModeResult", "BASELINE");
    } else if (anyFail || anyWarn) {
      Logger.recordOutput("TestModeResult", "FAIL");
    } else {
      Logger.recordOutput("TestModeResult", "PASS");
    }

    return "<!DOCTYPE html>\n"
        + "<html lang='en'>\n"
        + "<head>\n"
        + "<meta charset='UTF-8'>\n"
        + "<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
        + "<title>Robot Motor Test Report</title>\n"
        + "<style>\n"
        + "body{font-family:Arial,sans-serif;margin:0;background:#f5f5f5;color:#212121}\n"
        + ".banner{display:block;width:100%;padding:32px 28px;box-sizing:border-box;color:#fff;"
        + "font-size:2.6em;font-weight:900;text-align:center;letter-spacing:.06em;background:"
        + overallColor
        + ";margin-bottom:0}\n"
        + ".content{padding:28px}\n"
        + "h1{margin:0 0 6px}\n"
        + ".meta{color:#616161;margin:10px 0 22px;font-size:.88em}\n"
        + "table{border-collapse:collapse;width:100%;background:#fff;"
        + "box-shadow:0 1px 4px rgba(0,0,0,.14)}\n"
        + "th{background:#37474f;color:#fff;padding:10px 13px;text-align:left;white-space:nowrap}\n"
        + "td{padding:7px 13px;border-bottom:1px solid #e0e0e0;white-space:nowrap}\n"
        + "tr.sh td{background:#eceff1;font-weight:700;font-size:.9em;"
        + "color:#37474f;letter-spacing:.06em;padding:5px 13px}\n"
        + "tr:not(.sh):hover td{background:#f9fbe7}\n"
        + ".legend{margin-top:18px;font-size:.82em;color:#757575}\n"
        + ".dot{display:inline-block;width:11px;height:11px;border-radius:2px;"
        + "margin-right:3px;vertical-align:middle}\n"
        + "</style>\n"
        + "</head>\n"
        + "<body>\n"
        + "<div class='banner'>"
        + overallLabel
        + "</div>\n"
        + "<div class='content'>\n"
        + "<h1>Robot Motor Test Report</h1>\n"
        + "<div class='meta'>"
        + "Date: <b>"
        + date
        + "</b>&nbsp;&nbsp;|&nbsp;&nbsp;"
        + "Robot serial: <b>"
        + escHtml(robotSerial)
        + "</b>&nbsp;&nbsp;|&nbsp;&nbsp;"
        + "Git: <b>"
        + escHtml(sha8)
        + "</b>&nbsp;&nbsp;|&nbsp;&nbsp;"
        + "PASS&nbsp;"
        + passCount
        + "&nbsp; WARN&nbsp;"
        + warnCount
        + "&nbsp; FAIL/NO_BASELINE&nbsp;"
        + failCount
        + "</div>\n"
        + "<table>\n"
        + "<thead><tr>"
        + "<th>Motor</th>"
        + "<th>Baseline mean vel</th>"
        + "<th>Measured mean vel</th>"
        + "<th>Measured min vel</th>"
        + "<th>&Delta;%</th>"
        + "<th>Chart&nbsp;<small>(grey=baseline&nbsp;/&nbsp;color=measured)</small></th>"
        + "<th>Status</th>"
        + "</tr></thead>\n"
        + "<tbody>\n"
        + rows
        + "</tbody>\n"
        + "</table>\n"
        + "<div class='legend'>\n"
        + "Velocity units: <b>rad/s</b> for drive modules &nbsp;|&nbsp; "
        + "<b>RPS</b> for all other mechanisms.<br>\n"
        + "<span class='dot' style='background:#2e7d32'></span>PASS (within &plusmn;25%)"
        + "&ensp;<span class='dot' style='background:#e65100'></span>"
        + "WARN (25&ndash;50% below baseline, or &gt;30% above)"
        + "&ensp;<span class='dot' style='background:#c62828'></span>"
        + "FAIL (&gt;50% below or near-zero)"
        + "&ensp;<span class='dot' style='background:#757575'></span>NO_BASELINE\n"
        + "</div>\n"
        + "</div>\n"
        + "</body>\n"
        + "</html>\n";
  }

  /**
   * Writes the HTML string to the USB drive (preferred) or falls back to {@code
   * /home/lvuser/test-report.html}.
   */
  public static void writeToDisk(String html) {
    DateTimeFormatter fmt = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
    String ts = LocalDateTime.now().format(fmt);

    // Try USB drive first, then roboRIO home
    String[] candidates = {"/U/logs/test-report-" + ts + ".html", "/home/lvuser/test-report.html"};

    for (String path : candidates) {
      File dir = new File(path).getParentFile();
      if (dir != null && !dir.exists()) continue;
      try (FileWriter fw = new FileWriter(path)) {
        fw.write(html);
        System.out.println("[TestMode] Report written to: " + path);
        return;
      } catch (IOException e) {
        System.err.println("[TestMode] Could not write to " + path + ": " + e.getMessage());
      }
    }
    System.err.println("[TestMode] WARNING: Could not write report to any location.");
  }

  private static String statusColor(TestStatus status) {
    return switch (status) {
      case PASS -> "#2e7d32";
      case WARN -> "#f9cd08ff";
      case FAIL -> "#c62828";
      case NO_BASELINE -> "#757575";
    };
  }

  private static String escHtml(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }
}
