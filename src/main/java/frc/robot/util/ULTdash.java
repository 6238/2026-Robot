package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Publishes custom dashboard widget layouts to ULTdash via NT4.
 *
 * Usage:
 *   private final ULTdash dash = new ULTdash();
 *
 *   // In constructor / robotInit:
 *   dash.gauge("shooter", "/SmartDashboard/ShooterRPM", "Shooter RPM")
 *       .min(0).max(6000).unit("RPM").threshold(5500, "green")
 *       .at(0, 0).size(2, 2);
 *   dash.number("speed", "/SmartDashboard/Speed", "Speed m/s")
 *       .unit("m/s").decimals(2).at(2, 0).size(2, 1);
 *   dash.bool("enabled", "/FMSInfo/IsEnabled", "Enabled").at(4, 0).size(1, 1);
 *
 *   // In robotPeriodic():
 *   dash.periodic();
 */
public class ULTdash {
    private static final double PUBLISH_INTERVAL = 2.0;

    private final Map<String, Widget> widgets = new LinkedHashMap<>();
    private double lastPublish = Double.NEGATIVE_INFINITY;
    private int epoch = 1;
    private int lastWidgetHash = Integer.MIN_VALUE;

    // --- Factory methods ---

    public NumberWidget number(String id, String source, String label) {
        NumberWidget w = new NumberWidget(id, source, label);
        widgets.put(id, w);
        return w;
    }

    public GaugeWidget gauge(String id, String source, String label) {
        GaugeWidget w = new GaugeWidget(id, source, label);
        widgets.put(id, w);
        return w;
    }

    public BoolWidget bool(String id, String source, String label) {
        BoolWidget w = new BoolWidget(id, source, label);
        widgets.put(id, w);
        return w;
    }

    public StringWidget string(String id, String source, String label) {
        StringWidget w = new StringWidget(id, source, label);
        widgets.put(id, w);
        return w;
    }

    public GraphWidget graph(String id, String source, String label) {
        GraphWidget w = new GraphWidget(id, source, label);
        widgets.put(id, w);
        return w;
    }

    /** Call from robotPeriodic(). Re-publishes all widget definitions every 2 seconds. */
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastPublish < PUBLISH_INTERVAL) return;
        lastPublish = now;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        int currentHash = widgets.keySet().hashCode();
        if (currentHash != lastWidgetHash) {
            lastWidgetHash = currentHash;
            epoch++;
        }
        nt.getTable("dashboard").getEntry("epoch").setInteger(epoch);

        for (Widget w : widgets.values()) {
            NetworkTable t = nt.getTable("dashboard").getSubTable("widgets").getSubTable(w.id);
            t.getEntry("type").setString(w.type);
            t.getEntry("source").setString(w.source);
            t.getEntry("label").setString(w.label);
            t.getEntry("x").setInteger(w.x);
            t.getEntry("y").setInteger(w.y);
            t.getEntry("w").setInteger(w.w);
            t.getEntry("h").setInteger(w.h);
            t.getEntry("config").setString(w.buildConfig());
        }
    }

    // --- Widget base ---

    public abstract static class Widget {
        final String id;
        final String type;
        final String source;
        final String label;
        int x, y, w, h;

        Widget(String id, String type, String source, String label) {
            this.id = id;
            this.type = type;
            this.source = source;
            this.label = label;
        }

        abstract String buildConfig();
    }

    @SuppressWarnings("unchecked")
    public abstract static class BaseWidget<T extends BaseWidget<T>> extends Widget {
        BaseWidget(String id, String type, String source, String label) {
            super(id, type, source, label);
        }

        public T at(int col, int row) {
            this.x = col;
            this.y = row;
            return (T) this;
        }

        public T size(int w, int h) {
            this.w = w;
            this.h = h;
            return (T) this;
        }
    }

    // --- Widget types ---

    public static final class NumberWidget extends BaseWidget<NumberWidget> {
        private String unit = "";
        private int decimals = 2;

        NumberWidget(String id, String source, String label) {
            super(id, "number", source, label);
        }

        public NumberWidget unit(String unit) { this.unit = unit; return this; }
        public NumberWidget decimals(int d) { this.decimals = d; return this; }

        @Override
        String buildConfig() {
            return "{\"unit\":\"" + esc(unit) + "\",\"decimals\":" + decimals + "}";
        }
    }

    public static final class GaugeWidget extends BaseWidget<GaugeWidget> {
        private double min = 0;
        private double max = 100;
        private String unit = "";
        private final List<String> thresholds = new ArrayList<>();

        GaugeWidget(String id, String source, String label) {
            super(id, "gauge", source, label);
        }

        public GaugeWidget min(double v) { this.min = v; return this; }
        public GaugeWidget max(double v) { this.max = v; return this; }
        public GaugeWidget unit(String u) { this.unit = u; return this; }

        public GaugeWidget threshold(double value, String color) {
            thresholds.add("{\"value\":" + value + ",\"color\":\"" + esc(color) + "\"}");
            return this;
        }

        @Override
        String buildConfig() {
            return "{\"min\":" + min + ",\"max\":" + max
                    + ",\"unit\":\"" + esc(unit) + "\""
                    + ",\"thresholds\":[" + String.join(",", thresholds) + "]}";
        }
    }

    public static final class BoolWidget extends BaseWidget<BoolWidget> {
        BoolWidget(String id, String source, String label) {
            super(id, "bool", source, label);
        }

        @Override
        String buildConfig() { return "{}"; }
    }

    public static final class StringWidget extends BaseWidget<StringWidget> {
        StringWidget(String id, String source, String label) {
            super(id, "string", source, label);
        }

        @Override
        String buildConfig() { return "{}"; }
    }

    public static final class GraphWidget extends BaseWidget<GraphWidget> {
        private double min = 0;
        private double max = 100;
        private int windowSeconds = 10;
        private String unit = "";

        GraphWidget(String id, String source, String label) {
            super(id, "graph", source, label);
        }

        public GraphWidget min(double v) { this.min = v; return this; }
        public GraphWidget max(double v) { this.max = v; return this; }
        public GraphWidget windowSeconds(int s) { this.windowSeconds = s; return this; }
        public GraphWidget unit(String u) { this.unit = u; return this; }

        @Override
        String buildConfig() {
            return "{\"min\":" + min + ",\"max\":" + max
                    + ",\"windowSeconds\":" + windowSeconds
                    + ",\"unit\":\"" + esc(unit) + "\"}";
        }
    }

    // --- Util ---

    private static String esc(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
