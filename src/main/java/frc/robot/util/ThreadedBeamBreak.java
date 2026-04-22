package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Polls a beam-break DIO sensor at 250 Hz on a background thread. The latch is set on any triggered
 * reading; {@link #getTriggeredSinceLastCheck()} returns and resets it, letting 50 Hz IO loops
 * catch brief ball passages that would otherwise be missed between cycles.
 */
public class ThreadedBeamBreak extends Thread {
  private final DigitalInput sensor;
  private final boolean inverted;
  private final AtomicBoolean latch = new AtomicBoolean(false);

  private static final double POLL_HZ = 250.0;

  /**
   * @param port DIO port number
   * @param inverted true if the sensor output is HIGH when the beam is broken
   */
  public ThreadedBeamBreak(int port, boolean inverted) {
    this.sensor = new DigitalInput(port);
    this.inverted = inverted;
    setName("BeamBreak-DIO" + port);
    setDaemon(true);
  }

  private boolean isTriggered() {
    return inverted ? sensor.get() : !sensor.get();
  }

  @Override
  public void run() {
    while (!isInterrupted()) {
      if (isTriggered()) latch.set(true);
      try {
        Thread.sleep((long) (1000.0 / POLL_HZ));
      } catch (InterruptedException e) {
        interrupt();
      }
    }
  }

  /**
   * Returns true if the beam break was triggered at least once since the last call. Resets the
   * latch, so each 50 Hz cycle gets exactly one report.
   */
  public boolean getTriggeredSinceLastCheck() {
    return latch.getAndSet(false);
  }

  /** Real-time sensor state without consuming the latch. */
  public boolean isCurrentlyTriggered() {
    return isTriggered();
  }
}
