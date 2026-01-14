package frc.robot.util;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Seconds;

public class TimedSampleBuffer<T> {

  public record TimedSample<T>(Time timestamp, T value) {}

  private final Deque<TimedSample<T>> samples = new ArrayDeque<>();

  /**
   * Add a sample with an explicit timestamp.
   * Timestamp should be monotonic (non-decreasing) for pruning to work well.
   */
  public void add(Time timestamp, T value) {
    samples.addLast(new TimedSample<>(timestamp, value));
  }

  /**
   * Remove samples older than (now - window).
   */
  public void pruneOlderThan(Time now, Time window) {
    double cutoffSec = now.in(Seconds) - window.in(Seconds);
    while (!samples.isEmpty() && samples.peekFirst().timestamp().in(Seconds) < cutoffSec) {
      samples.removeFirst();
    }
  }

  /**
   * Returns a copy of samples whose timestamp is within the last `window` seconds,
   * relative to the provided `now`. Old samples are not removed unless you call prune.
   */
  public List<TimedSample<T>> getLast(Time now, Time window) {
    double cutoffSec = now.in(Seconds) - window.in(Seconds);

    // Walk from newest backwards until outside window.
    List<TimedSample<T>> outReversed = new ArrayList<>();
    for (var it = samples.descendingIterator(); it.hasNext();) {
      TimedSample<T> s = it.next();
      if (s.timestamp().in(Seconds) < cutoffSec) break;
      outReversed.add(s);
    }

    // Reverse to chronological order (oldest->newest)
    List<TimedSample<T>> out = new ArrayList<>(outReversed.size());
    for (int i = outReversed.size() - 1; i >= 0; i--) {
      out.add(outReversed.get(i));
    }
    return out;
  }

  /**
   * Convenience: get the last window relative to the most recent sample's timestamp.
   */
  public List<TimedSample<T>> getLast(Time window) {
    TimedSample<T> last = samples.peekLast();
    if (last == null) return List.of();
    return getLast(last.timestamp(), window);
  }

  public TimedSample<T> getLatest() {
    return samples.peekLast();
  }

  public int size() {
    return samples.size();
  }

  public void clear() {
    samples.clear();
  }
}
