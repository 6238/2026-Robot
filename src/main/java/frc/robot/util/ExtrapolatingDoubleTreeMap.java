package frc.robot.util;

import java.util.TreeMap;

/**
 * A map of doubles to doubles that supports both interpolation between entries and linear
 * extrapolation beyond the endpoints. When the queried key is within the range of stored entries,
 * behavior is identical to WPILib's InterpolatingDoubleTreeMap. When outside the range, the slope
 * of the two nearest endpoint entries is used to extrapolate.
 */
public class ExtrapolatingDoubleTreeMap {
  private final TreeMap<Double, Double> map = new TreeMap<>();

  /** Inserts a key-value pair. */
  public void put(double key, double value) {
    map.put(key, value);
  }

  /**
   * Returns the interpolated or extrapolated value for the given key.
   *
   * @param key the input key
   * @return interpolated value if key is within range, extrapolated value if outside range, or the
   *     sole entry's value if only one entry exists
   */
  public double get(double key) {
    if (map.isEmpty()) {
      return 0.0;
    }

    Double exactMatch = map.get(key);
    if (exactMatch != null) {
      return exactMatch;
    }

    if (map.size() == 1) {
      return map.firstEntry().getValue();
    }

    Double lowerKey = map.floorKey(key);
    Double upperKey = map.ceilingKey(key);

    // Extrapolate below the minimum
    if (lowerKey == null) {
      double k0 = map.firstKey();
      double k1 = map.higherKey(k0);
      double v0 = map.get(k0);
      double v1 = map.get(k1);
      return interpolate(k0, v0, k1, v1, key);
    }

    // Extrapolate above the maximum
    if (upperKey == null) {
      double k1 = map.lastKey();
      double k0 = map.lowerKey(k1);
      double v0 = map.get(k0);
      double v1 = map.get(k1);
      return interpolate(k0, v0, k1, v1, key);
    }

    // Interpolate between two bracketing entries
    double v0 = map.get(lowerKey);
    double v1 = map.get(upperKey);
    return interpolate(lowerKey, v0, upperKey, v1, key);
  }

  /** Clears all entries. */
  public void clear() {
    map.clear();
  }

  /** Returns the number of entries. */
  public int size() {
    return map.size();
  }

  private static double interpolate(double x0, double y0, double x1, double y1, double x) {
    if (x0 == x1) {
      return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}