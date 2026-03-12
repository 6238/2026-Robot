package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class HopperTest {
  static final double DELTA = 1e-6;

  HopperIO mockHopperIO;
  Hopper hopper;

  @BeforeEach
  void setup() {
    mockHopperIO = Mockito.mock(HopperIO.class);
    // Default: motors connected so Alert.set(true) is never triggered without HAL
    doAnswer(
            inv -> {
              HopperIOInputs inputs = inv.getArgument(0);
              inputs.indexerTalonConnected = true;
              inputs.topIndexerTalonConnected = true;
              return null;
            })
        .when(mockHopperIO)
        .updateInputs(any(HopperIOInputs.class));
    hopper = new Hopper(mockHopperIO);
  }

  @AfterEach
  void tearDown() {
    hopper = null;
  }

  // ── spinIndexer / stopIndexer ─────────────────────────────────────────────

  @Test
  void spinIndexer_callsSetIndexerVoltage() {
    hopper.spinIndexer().initialize();
    verify(mockHopperIO).setIndexerVoltage(any(Voltage.class));
  }

  @Test
  void stopIndexer_setsIndexerVoltageToZero() {
    hopper.stopIndexer().initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockHopperIO).setIndexerVoltage(captor.capture());
    assertEquals(0.0, captor.getValue().in(Volts), DELTA);
  }

  // ── spinTopIndexer / stopTopIndexer ─────────────────────────────────────

  @Test
  void spinTopIndexer_callsSetTopIndexerVoltage() {
    hopper.spinTopIndexer().initialize();
    verify(mockHopperIO).setTopIndexerVoltage(any(Voltage.class));
  }

  @Test
  void stopTopIndexer_setsTopIndexerVoltageToZero() {
    hopper.stopTopIndexer().initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockHopperIO).setTopIndexerVoltage(captor.capture());
    assertEquals(0.0, captor.getValue().in(Volts), DELTA);
  }

  // ── spinFullIndexer / stopFullIndexer ────────────────────────────────────

  @Test
  void spinFullIndexer_setsBothMotors() {
    hopper.spinFullIndexer().initialize();
    verify(mockHopperIO).setIndexerVoltage(any(Voltage.class));
    verify(mockHopperIO).setTopIndexerVoltage(any(Voltage.class));
  }

  @Test
  void stopFullIndexer_setsBothVoltagesToZero() {
    hopper.stopFullIndexer().initialize();

    ArgumentCaptor<Voltage> indexerCaptor = ArgumentCaptor.forClass(Voltage.class);
    ArgumentCaptor<Voltage> topCaptor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockHopperIO).setIndexerVoltage(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerVoltage(topCaptor.capture());

    assertEquals(0.0, indexerCaptor.getValue().in(Volts), DELTA);
    assertEquals(0.0, topCaptor.getValue().in(Volts), DELTA);
  }

  @Test
  void spinFullIndexerWithParams_setsExactVoltages() {
    hopper.spinFullIndexer(5.0, 7.0).initialize();

    ArgumentCaptor<Voltage> indexerCaptor = ArgumentCaptor.forClass(Voltage.class);
    ArgumentCaptor<Voltage> topCaptor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockHopperIO).setIndexerVoltage(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerVoltage(topCaptor.capture());

    assertEquals(5.0, indexerCaptor.getValue().in(Volts), DELTA);
    assertEquals(7.0, topCaptor.getValue().in(Volts), DELTA);
  }

  @Test
  void spinFullIndexerWithParams_negative_setsNegativeVoltages() {
    hopper.spinFullIndexer(-3.0, -5.0).initialize();

    ArgumentCaptor<Voltage> indexerCaptor = ArgumentCaptor.forClass(Voltage.class);
    ArgumentCaptor<Voltage> topCaptor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockHopperIO).setIndexerVoltage(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerVoltage(topCaptor.capture());

    assertEquals(-3.0, indexerCaptor.getValue().in(Volts), DELTA);
    assertEquals(-5.0, topCaptor.getValue().in(Volts), DELTA);
  }

  // ── periodic ─────────────────────────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    hopper.periodic();
    verify(mockHopperIO).updateInputs(any(HopperIOInputs.class));
  }

  @Test
  void periodic_callsUpdateInputs_withCorrectInputsObject() {
    // Verify IO receives the inputs object and can populate it
    doAnswer(
            inv -> {
              HopperIOInputs inputs = inv.getArgument(0);
              inputs.indexerTalonConnected = true;
              inputs.topIndexerTalonConnected = true;
              return null;
            })
        .when(mockHopperIO)
        .updateInputs(any(HopperIOInputs.class));

    hopper.periodic(); // should not throw

    verify(mockHopperIO).updateInputs(any(HopperIOInputs.class));
  }
}
