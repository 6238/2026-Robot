package frc.robot.subsystems.objectdetection;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class BallDetection implements StructSerializable {

  public static final BallDetectionStruct struct = new BallDetectionStruct();

  private final double x;
  private final double y;
  private final double distance;

  public BallDetection(double x, double y, double distance) {
    this.x = x;
    this.y = y;
    this.distance = distance;
  }

  public double x() {
    return x;
  }

  public double y() {
    return y;
  }

  public double distance() {
    return distance;
  }

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
      return new BallDetection(bb.getDouble(), bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, BallDetection value) {
      bb.putDouble(value.x());
      bb.putDouble(value.y());
      bb.putDouble(value.distance());
    }
  }
}
