package frc.robot.subsystems.objectdetection;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * A single ball detection published by the Jetson coprocessor over NetworkTables.
 *
 * <p>Coordinate frame (camera-relative):
 *
 * <ul>
 *   <li>{@code x} — lateral offset in meters; right of camera center is positive.
 *   <li>{@code y} — forward distance in meters; always positive (ball is in front of camera).
 *   <li>{@code distance} — Euclidean distance to the ball in meters.
 * </ul>
 *
 * <p>The Jetson should publish an array of these structs to the NetworkTables topic {@code
 * "objectdetection/balls"} using the schema {@code "double x;double y;double distance"}.
 */
public record BallDetection(double x, double y, double distance) implements StructSerializable {

  public static final BallDetectionStruct struct = new BallDetectionStruct();

  public static final class BallDetectionStruct implements Struct<BallDetection> {
    @Override
    public Class<BallDetection> getTypeClass() {
      return BallDetection.class;
    }

    @Override
    public String getTypeName() {
      return "BallDetection";
    }

    @Override
    public String getTypeString() {
      return "struct:BallDetection";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 3;
    }

    @Override
    public String getSchema() {
      return "double x;double y;double distance";
    }

    @Override
    public BallDetection unpack(ByteBuffer bb) {
      double x = bb.getDouble();
      double y = bb.getDouble();
      double distance = bb.getDouble();
      return new BallDetection(x, y, distance);
    }

    @Override
    public void pack(ByteBuffer bb, BallDetection value) {
      bb.putDouble(value.x());
      bb.putDouble(value.y());
      bb.putDouble(value.distance());
    }
  }
}
