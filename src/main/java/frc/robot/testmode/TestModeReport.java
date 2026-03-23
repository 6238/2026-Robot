package frc.robot.testmode;

import frc.robot.testmode.TestModeRunner.MotorResult;
import frc.robot.testmode.TestModeRunner.TestStatus;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

/** Generates and writes a self-contained HTML motor-test report. */
public class TestModeReport {

  /**
   * Builds a self-contained HTML string from the completed motor test results.
   *
   * @param results list of per-motor results from {@link TestModeRunner}
   * @param robotSerial roboRIO serial number
   * @param gitSha full git SHA (will be truncated to 8 chars in display)
   */
  public static String generate(List<MotorResult> results, String robotSerial, String gitSha) {
    String date = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"));

    long passCount = results.stream().filter(r -> r.status() == TestStatus.PASS).count();
    long warnCount = results.stream().filter(r -> r.status() == TestStatus.WARN).count();
    long failCount =
        results.stream()
            .filter(r -> r.status() == TestStatus.FAIL || r.status() == TestStatus.NO_BASELINE)
            .count();

    String overallLabel;
    String overallColor;
    if (failCount > 0) {
      overallLabel = "FAIL &mdash; " + failCount + " issue(s)";
      overallColor = "#c62828";
    } else if (warnCount > 0) {
      overallLabel = "WARN &mdash; " + warnCount + " warning(s)";
      overallColor = "#e65100";
    } else {
      overallLabel = "PASS &mdash; all motors nominal";
      overallColor = "#2e7d32";
    }

    String sha8 = gitSha.length() >= 8 ? gitSha.substring(0, 8) : gitSha;

    // Build table rows grouped by step
    List<String> steps = results.stream().map(MotorResult::step).distinct().toList();
    StringBuilder rows = new StringBuilder();

    for (String step : steps) {
      List<MotorResult> stepResults = results.stream().filter(r -> r.step().equals(step)).toList();

      rows.append("<tr class='sh'><td colspan='7'>").append(escHtml(step)).append("</td></tr>\n");

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

        // Inline SVG bar: grey = baseline, coloured = measured
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

    return "<!DOCTYPE html>\n"
        + "<html lang='en'>\n"
        + "<head>\n"
        + "<meta charset='UTF-8'>\n"
        + "<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
        + "<title>Robot Motor Test Report</title>\n"
        + "<style>\n"
        + "body{font-family:Arial,sans-serif;margin:28px;background:#f5f5f5;color:#212121}\n"
        + "h1{margin-bottom:6px}\n"
        + ".badge{display:inline-block;padding:7px 18px;border-radius:5px;color:#fff;"
        + "font-size:1.05em;font-weight:700;background:"
        + overallColor
        + "}\n"
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
        + "<h1>Robot Motor Test Report</h1>\n"
        + "<div class='badge'>"
        + overallLabel
        + "</div>\n"
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
      case WARN -> "#e65100";
      case FAIL -> "#c62828";
      case NO_BASELINE -> "#757575";
    };
  }

  private static String escHtml(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }
}
