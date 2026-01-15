package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class HX711Sensor implements AutoCloseable {
    private DigitalInput dout;
    private DigitalOutput sck;

    private final AtomicLong latestCounts = new AtomicLong(0);
    private final AtomicBoolean hasNewSample = new AtomicBoolean(false);

    // Calibration parameters
    public final long offsetCounts = 0; // Tare offset, set externally as needed
    public final long scaleFactor = 1; // Scale factor, set externally as needed

    private static final int TOTAL_PULSES = 25; // Channel A Gain of 128

    private static final Time CLOCK_HIGH = Microseconds.of(1);
    private static final Time CLOCK_LOW = Microseconds.of(1);

    public HX711Sensor(int doutPin, int sckPin) {
        dout = new DigitalInput(doutPin);
        sck = new DigitalOutput(sckPin);
        sck.set(false);
    }

    // HX711 sets DOUT low when data is ready
    public boolean isReady() {
        return !dout.get();
    }
    
    public boolean consumeNewSampleFlag() {
        return hasNewSample.getAndSet(false);
    }

    /**
     * Call periodically to read new data from the HX711 if available
     */
    public void update() {
        if (!isReady()) return;

        int value = 0;

        // Read 24 bits MSB-first
        for (int i = 0; i < 24; i++) {
            pulseClock();
            value = (value << 1) | (dout.get() ? 1 : 0);
        }

        // Extra pulses to select gain/channel for next conversion
        for (int i = 24; i < TOTAL_PULSES; i++) {
            pulseClock();
        }

        // Sign-extend 24-bit two's complement to 32-bit int
        if ((value & 0x800000) != 0) {
            value |= 0xFF000000;
        }

        // Apply calibration parameters
        value = (value - (int)offsetCounts) / (int)scaleFactor;

        latestCounts.set(value);
        hasNewSample.set(true);
    }

    private void pulseClock() {
        sck.set(true);
        Timer.delay(CLOCK_HIGH.in(Seconds));
        sck.set(false);
        Timer.delay(CLOCK_LOW.in(Seconds));
    }

    @Override
    public void close() {
        dout.close();
        sck.close();
    }
}