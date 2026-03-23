package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIO.IntakeRollerIOInputs;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class IntakeRollerTest {
  static final double DELTA = 1e-6;

  IntakeRollerIO mockIntakeRollerIO;
  IntakeRoller intakeRoller;

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void shutdownHAL() {
    SimHooks.resumeTiming();
  }

  @BeforeEach
  void setup() {
    SimHooks.restartTiming();
    mockIntakeRollerIO = Mockito.mock(IntakeRollerIO.class);
    doAnswer(
            inv -> {
              IntakeRollerIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              return null;
            })
        .when(mockIntakeRollerIO)
        .updateInputs(any(IntakeRollerIOInputs.class));
    intakeRoller = new IntakeRoller(mockIntakeRollerIO);
  }

  @AfterEach
  void tearDown() {
    intakeRoller = null;
  }

  // ── spinIntake ────────────────────────────────────────────────────────────

  @Test
  void spinIntake_setsIntakeVelocity() {
    intakeRoller.spinIntake().initialize();
    verify(mockIntakeRollerIO).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void spinIntake_setsPositiveVelocity() {
    intakeRoller.spinIntake().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockIntakeRollerIO).setIntakeVelocity(captor.capture());
    assertTrue(captor.getValue().in(RotationsPerSecond) > 0);
  }

  // ── stopIntake ────────────────────────────────────────────────────────────

  @Test
  void stopIntake_setsIntakeVoltageToZero() {
    intakeRoller.stopIntake().initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeRollerIO).setIntakeVoltage(captor.capture());
    assertEquals(0.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void stopIntake_afterSpinning_stopsCompletely() {
    intakeRoller.spinIntake().initialize();
    intakeRoller.stopIntake().initialize();

    ArgumentCaptor<Voltage> voltageCaptor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeRollerIO).setIntakeVoltage(voltageCaptor.capture());
    assertEquals(0.0, voltageCaptor.getValue().in(Volts), DELTA);
  }

  // ── reverseIntake ─────────────────────────────────────────────────────────

  @Test
  void reverseIntake_setsNegativeVelocity() {
    intakeRoller.reverseIntake().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockIntakeRollerIO).setIntakeVelocity(captor.capture());
    assertTrue(captor.getValue().in(RotationsPerSecond) < 0);
  }

  // ── periodic ─────────────────────────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    intakeRoller.periodic();
    verify(mockIntakeRollerIO).updateInputs(any(IntakeRollerIOInputs.class));
  }

  // ── jam prevention ────────────────────────────────────────────────────────

  @Test
  void jam_detection_triggers_reversal() {
    intakeRoller.spinIntake().initialize();
    setupStallInputs();

    intakeRoller.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intakeRoller.periodic();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeRollerIO, atLeastOnce()).setIntakeVoltage(captor.capture());
    double minVoltage =
        captor.getAllValues().stream().mapToDouble(v -> v.in(Volts)).min().orElse(0.0);
    assertEquals(-12.0, minVoltage, DELTA);
  }

  @Test
  void jam_reversal_exits_when_velocity_recovers() {
    intakeRoller.spinIntake().initialize();
    setupStallInputs();
    intakeRoller.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intakeRoller.periodic();

    setupRecoveredInputs();
    SimHooks.stepTiming(IntakeConstants.JAM_REVERSE_MIN_DURATION_SECONDS + 0.02);
    intakeRoller.periodic();

    verify(mockIntakeRollerIO, atLeastOnce()).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void jam_not_triggered_when_idle() {
    setupStallInputs();
    intakeRoller.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intakeRoller.periodic();

    verify(mockIntakeRollerIO, never()).setIntakeVoltage(argThat(v -> v.in(Volts) < 0));
  }

  @Test
  void stopIntake_clears_jam_reversing_state() {
    intakeRoller.spinIntake().initialize();
    setupStallInputs();
    intakeRoller.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intakeRoller.periodic();

    intakeRoller.stopIntake().initialize();
    Mockito.clearInvocations(mockIntakeRollerIO);

    intakeRoller.spinIntake().initialize();

    verify(mockIntakeRollerIO).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void onJamDetected_callback_invoked() {
    AtomicBoolean callbackFired = new AtomicBoolean(false);
    intakeRoller = new IntakeRoller(mockIntakeRollerIO, () -> callbackFired.set(true));

    intakeRoller.spinIntake().initialize();
    setupStallInputs();
    intakeRoller.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intakeRoller.periodic();

    assertTrue(callbackFired.get(), "onJamDetected callback should have been invoked");
  }

  // ── helpers ───────────────────────────────────────────────────────────────

  private void setupStallInputs() {
    doAnswer(
            inv -> {
              IntakeRollerIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              inputs.intakeVelocity = RotationsPerSecond.of(1.0);
              inputs.intakeSupplyCurrent = Amps.of(60.0);
              return null;
            })
        .when(mockIntakeRollerIO)
        .updateInputs(any(IntakeRollerIOInputs.class));
  }

  private void setupRecoveredInputs() {
    doAnswer(
            inv -> {
              IntakeRollerIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              inputs.intakeVelocity = RotationsPerSecond.of(20.0);
              inputs.intakeSupplyCurrent = Amps.of(10.0);
              return null;
            })
        .when(mockIntakeRollerIO)
        .updateInputs(any(IntakeRollerIOInputs.class));
  }
}
