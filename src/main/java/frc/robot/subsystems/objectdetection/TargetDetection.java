package frc.robot.subsystems.objectdetection;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class TargetDetection implements StructSerializable {
  public final double dx;
  public final double dy;
  public final double area;
  public final double confidence;
  public final double timestamp;

  public TargetDetection(double dx, double dy, double area, double confidence, double timestamp) {
    this.dx = dx;
    this.dy = dy;
    this.area = area;
    this.confidence = confidence;
    this.timestamp = timestamp;
  }

  public static final Struct<TargetDetection> struct =
      new Struct<>() {
        @Override
        public Class<TargetDetection> getTypeClass() {
          return TargetDetection.class;
        }

        @Override
        public String getTypeName() {
          return "struct:TargetDetection";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 5;
        }

        @Override
        public String getSchema() {
          return "double dx; double dy; double area; double confidence; double timestamp";
        }

        @Override
        public TargetDetection unpack(ByteBuffer bb) {
          double dx = bb.getDouble();
          double dy = bb.getDouble();
          double area = bb.getDouble();
          double conf = bb.getDouble();
          double time = bb.getDouble();
          return new TargetDetection(dx, dy, area, conf, time);
        }

        @Override
        public void pack(ByteBuffer bb, TargetDetection value) {
          bb.putDouble(value.dx);
          bb.putDouble(value.dy);
          bb.putDouble(value.area);
          bb.putDouble(value.confidence);
          bb.putDouble(value.timestamp);
        }
      };
}
