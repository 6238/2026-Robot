package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.measure.*;
import java.util.ArrayList;
import java.util.Collection;

/** Class to manage bulk refreshing device status signals. */
public class BetterStatusSignalCollection {
  /** Signals stored by this collection */
  protected final ArrayList<BaseStatusSignal> _signals = new ArrayList<>();

  /**
   * Creates a new collection of status signals, optionally adding the provided signals to the
   * collection.
   *
   * @param signals Signals to add
   */
  public BetterStatusSignalCollection(BaseStatusSignal... signals) {
    addSignals(signals);
  }
  /**
   * Creates a new collection of status signals, adding the provided list of signals to the
   * collection.
   *
   * @param signals Signals to add
   */
  public BetterStatusSignalCollection(Collection<BaseStatusSignal> signals) {
    addSignals(signals);
  }

  /**
   * Adds the provided signals to the collection.
   *
   * @param signals Signals to add
   */
  public final void addSignals(BaseStatusSignal... signals) {
    for (var signal : signals) {
      _signals.add(signal);
    }
  }
  /**
   * Adds the provided signals to the collection.
   *
   * @param signals Signals to add
   */
  public final void addSignals(Collection<BaseStatusSignal> signals) {
    for (var signal : signals) {
      _signals.add(signal);
    }
  }

  /**
   * Waits for new data on all signals up to timeout. This API is typically used with CANivore Bus
   * signals as they will be synced using the CANivore Timesync feature and arrive simultaneously.
   * Signals on a roboRIO bus cannot be synced and may require a significantly longer blocking call
   * to receive all signals.
   *
   * <p>Note that CANivore Timesync requires Phoenix Pro.
   *
   * <p>This can also be used with a timeout of zero to refresh many signals at once, which is
   * faster than calling refresh() on every signal. This is equivalent to calling {@link
   * #refreshAll}.
   *
   * <p>If a signal arrives multiple times while waiting, such as when *not* using CANivore
   * Timesync, the newest signal data is fetched. Additionally, if this function times out, the
   * newest signal data is fetched for all signals (when possible). We recommend checking the
   * individual status codes using {@link BaseStatusSignal#getStatus()} when this happens.
   *
   * @param timeoutSeconds Maximum time to wait for new data in seconds. Pass zero to refresh all
   *     signals without blocking.
   * @return An InvalidParamValue if this signal collection is empty, InvalidNetwork if signals are
   *     on different CAN bus networks, RxTimeout if it took longer than timeoutSeconds to receive
   *     all the signals, MultiSignalNotSupported if using the roboRIO bus with more than one signal
   *     and a non-zero timeout. An OK status code means that all signals arrived within
   *     timeoutSeconds and they are all OK.
   *     <p>Any other value represents the StatusCode of the first failed signal. Call getStatus()
   *     on each signal to determine which ones failed.
   */
  public final StatusCode waitForAll(double timeoutSeconds) {
    return BaseStatusSignal.waitForAll(timeoutSeconds, _signals);
  }

  /**
   * Performs a non-blocking refresh on all signals.
   *
   * <p>This provides a performance improvement over separately calling refresh() on each signal.
   *
   * @return An InvalidParamValue if this signal collection is empty, InvalidNetwork if signals are
   *     on different CAN bus networks. An OK status code means that all signals are OK.
   *     <p>Any other value represents the StatusCode of the first failed signal. Call getStatus()
   *     on each signal to determine which ones failed.
   */
  public final StatusCode refreshAll() {
    return BaseStatusSignal.refreshAll(_signals);
  }

  /**
   * Sets the update frequency of all status signals to the provided common frequency.
   *
   * <p>A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
   * frequency is 4 Hz, and the maximum is 1000 Hz.
   *
   * <p>If other StatusSignals in the same status frame have been set to an update frequency, the
   * fastest requested update frequency will be applied to the frame.
   *
   * <p>This will wait up to 0.100 seconds (100ms) for each signal.
   *
   * @param frequencyHz Rate to publish the signal in Hz
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public final StatusCode setUpdateFrequencyForAll(double frequencyHz) {
    return BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz, _signals);
  }

  /**
   * Sets the update frequency of all status signals to the provided common frequency.
   *
   * <p>A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
   * frequency is 4 Hz, and the maximum is 1000 Hz.
   *
   * <p>If other StatusSignals in the same status frame have been set to an update frequency, the
   * fastest requested update frequency will be applied to the frame.
   *
   * <p>This will wait up to 0.100 seconds (100ms) for each signal.
   *
   * @param frequency Rate to publish the signal
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public final StatusCode setUpdateFrequencyForAll(Frequency frequency) {
    return BaseStatusSignal.setUpdateFrequencyForAll(frequency, _signals);
  }

  /**
   * Checks if all signals have an OK error code.
   *
   * @return True if all are good, false otherwise
   */
  public final boolean isAllGood() {
    return BaseStatusSignal.isAllGood(_signals);
  }

  /**
   * Checks if all signals have an OK error code.
   *
   * @return True if all are good, false otherwise
   */
  public final ArrayList<BaseStatusSignal> getAllSignals() {
    return _signals;
  }

  public final String getBadSignalsString() {
    StringBuilder badSignals = new StringBuilder();
    for (var signal : _signals) {
      if (signal.getStatus() != StatusCode.OK) {
        if (badSignals.length() > 0) {
          badSignals.append(", ");
        }
        badSignals
            .append(signal.getName())
            .append(" (")
            .append(signal.getStatus().toString())
            .append(")");
      }
    }
    return badSignals.toString();
  }
}
