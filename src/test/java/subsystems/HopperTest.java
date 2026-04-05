package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
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
  void spinIndexer_callsSetIndexerSpeed() {
    hopper.spinIndexer().initialize();
    verify(mockHopperIO).setIndexerSpeed(any(AngularVelocity.class));
  }

  @Test
  void stopIndexer_setsIndexerSpeedToZero() {
    hopper.stopIndexer().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockHopperIO).setIndexerSpeed(captor.capture());
    assertEquals(0.0, captor.getValue().in(RotationsPerSecond), DELTA);
  }

  // ── spinTopIndexer / stopTopIndexer ─────────────────────────────────────

  @Test
  void spinTopIndexer_callsSetTopIndexerSpeed() {
    hopper.spinTopIndexer().initialize();
    verify(mockHopperIO).setTopIndexerSpeed(any(AngularVelocity.class));
  }

  @Test
  void stopTopIndexer_setsTopIndexerSpeedToZero() {
    hopper.stopTopIndexer().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockHopperIO).setTopIndexerSpeed(captor.capture());
    assertEquals(0.0, captor.getValue().in(RotationsPerSecond), DELTA);
  }

  // ── spinFullIndexer / stopFullIndexer ────────────────────────────────────

  @Test
  void spinFullIndexer_setsBothMotors() {
    hopper.spinFullIndexer().initialize();
    verify(mockHopperIO).setIndexerSpeed(any(AngularVelocity.class));
    verify(mockHopperIO).setTopIndexerSpeed(any(AngularVelocity.class));
  }

  @Test
  void stopFullIndexer_setsBothSpeedsToZero() {
    hopper.stopFullIndexer().initialize();

    ArgumentCaptor<AngularVelocity> indexerCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    ArgumentCaptor<AngularVelocity> topCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockHopperIO).setIndexerSpeed(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerSpeed(topCaptor.capture());

    assertEquals(0.0, indexerCaptor.getValue().in(RotationsPerSecond), DELTA);
    assertEquals(0.0, topCaptor.getValue().in(RotationsPerSecond), DELTA);
  }

  @Test
  void spinFullIndexerWithParams_setsExactSpeeds() {
    hopper.spinFullIndexer(RotationsPerSecond.of(5.0), RotationsPerSecond.of(7.0)).initialize();

    ArgumentCaptor<AngularVelocity> indexerCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    ArgumentCaptor<AngularVelocity> topCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockHopperIO).setIndexerSpeed(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerSpeed(topCaptor.capture());

    assertEquals(5.0, indexerCaptor.getValue().in(RotationsPerSecond), DELTA);
    assertEquals(7.0, topCaptor.getValue().in(RotationsPerSecond), DELTA);
  }

  @Test
  void spinFullIndexerWithParams_negative_setsNegativeSpeeds() {
    hopper.spinFullIndexer(RotationsPerSecond.of(-3.0), RotationsPerSecond.of(-5.0)).initialize();

    ArgumentCaptor<AngularVelocity> indexerCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    ArgumentCaptor<AngularVelocity> topCaptor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockHopperIO).setIndexerSpeed(indexerCaptor.capture());
    verify(mockHopperIO).setTopIndexerSpeed(topCaptor.capture());

    assertEquals(-3.0, indexerCaptor.getValue().in(RotationsPerSecond), DELTA);
    assertEquals(-5.0, topCaptor.getValue().in(RotationsPerSecond), DELTA);
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
