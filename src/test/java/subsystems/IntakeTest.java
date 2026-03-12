package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class IntakeTest {
  static final double DELTA = 1e-6;

  IntakeIO mockIntakeIO;
  Intake intake;

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
    mockIntakeIO = Mockito.mock(IntakeIO.class);
    // Default: motors connected so Alert.set(true) is never triggered without HAL
    doAnswer(
            inv -> {
              IntakeIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              inputs.intakeArmTalonConnected = true;
              return null;
            })
        .when(mockIntakeIO)
        .updateInputs(any(IntakeIOInputs.class));
    intake = new Intake(mockIntakeIO);
  }

  @AfterEach
  void tearDown() {
    intake = null;
  }

  // ── spinIntake ────────────────────────────────────────────────────────────

  @Test
  void spinIntake_setsIntakeVelocity() {
    intake.spinIntake().initialize();
    verify(mockIntakeIO).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void spinIntake_setsPositiveVelocity() {
    intake.spinIntake().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockIntakeIO).setIntakeVelocity(captor.capture());
    assertTrue(captor.getValue().in(RotationsPerSecond) > 0);
  }

  // ── stopIntake ────────────────────────────────────────────────────────────

  @Test
  void stopIntake_setsIntakeVoltageToZero() {
    intake.stopIntake().initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeIO).setIntakeVoltage(captor.capture());
    assertEquals(0.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void stopIntake_afterSpinning_stopsCompletely() {
    // Spin first, then stop
    intake.spinIntake().initialize();
    intake.stopIntake().initialize();

    // Last voltage command should be 0V
    ArgumentCaptor<Voltage> voltageCaptor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeIO).setIntakeVoltage(voltageCaptor.capture());
    assertEquals(0.0, voltageCaptor.getValue().in(Volts), DELTA);
  }

  // ── reverseIntake ─────────────────────────────────────────────────────────

  @Test
  void reverseIntake_setsNegativeVelocity() {
    intake.reverseIntake().initialize();

    ArgumentCaptor<AngularVelocity> captor = ArgumentCaptor.forClass(AngularVelocity.class);
    verify(mockIntakeIO).setIntakeVelocity(captor.capture());
    assertTrue(captor.getValue().in(RotationsPerSecond) < 0);
  }

  // ── setIntakeAngle ────────────────────────────────────────────────────────

  @Test
  void setIntakeAngle_callsSetIntakePosition() {
    intake.setIntakeAngle(() -> Degrees.of(45.0)).initialize();
    verify(mockIntakeIO).setIntakePosition(any(Angle.class));
  }

  @Test
  void setIntakeAngle_setsCorrectAngle() {
    Angle targetAngle = Degrees.of(45.0);
    intake.setIntakeAngle(() -> targetAngle).initialize();

    ArgumentCaptor<Angle> captor = ArgumentCaptor.forClass(Angle.class);
    verify(mockIntakeIO).setIntakePosition(captor.capture());
    assertEquals(45.0, captor.getValue().in(Degrees), DELTA);
  }

  @Test
  void setIntakeAngle_updatesTargetAngleField() {
    Angle targetAngle = Degrees.of(-35.0);
    intake.setIntakeAngle(() -> targetAngle).initialize();
    assertEquals(-35.0, intake.targetAngle.in(Degrees), DELTA);
  }

  // ── periodic ─────────────────────────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    intake.periodic();
    verify(mockIntakeIO).updateInputs(any(IntakeIOInputs.class));
  }

  // ── jam prevention ────────────────────────────────────────────────────────

  @Test
  void jam_detection_triggers_reversal() {
    // Spin intake so desired velocity > 0 (jam condition requires this)
    intake.spinIntake().initialize();

    // Mock stall condition: velocity below threshold AND current above threshold
    setupStallInputs();

    // First periodic at t=0: debounce starts
    intake.periodic();

    // Advance time past the debounce window (STALL_DEBOUNCE_SECONDS = 0.15s)
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);

    // Second periodic: debounce expires — jam reversal should trigger
    intake.periodic();

    // Verify -12V reversal was applied
    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockIntakeIO, atLeastOnce()).setIntakeVoltage(captor.capture());
    double minVoltage =
        captor.getAllValues().stream().mapToDouble(v -> v.in(Volts)).min().orElse(0.0);
    assertEquals(-12.0, minVoltage, DELTA);
  }

  @Test
  void jam_reversal_exits_when_velocity_recovers() {
    // Trigger jam reversal
    intake.spinIntake().initialize();
    setupStallInputs();
    intake.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intake.periodic(); // jam now active

    // Now mock velocity recovery (speed returns above threshold)
    setupRecoveredInputs();

    // Advance past the minimum jam reverse duration
    SimHooks.stepTiming(IntakeConstants.JAM_REVERSE_MIN_DURATION_SECONDS + 0.02);
    intake.periodic(); // should exit jam reversal and restore velocity command

    // After recovery, setIntakeVelocity should be called again
    verify(mockIntakeIO, atLeastOnce()).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void jam_not_triggered_when_idle() {
    // desiredVelocity = 0 (not spinning) — stall condition requires positive desired velocity
    setupStallInputs();
    intake.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intake.periodic();

    // Should NOT apply -12V (no jam detection when not spinning)
    verify(mockIntakeIO, never()).setIntakeVoltage(argThat(v -> v.in(Volts) < 0));
  }

  @Test
  void stopIntake_clears_jam_reversing_state() {
    // Trigger jam reversal
    intake.spinIntake().initialize();
    setupStallInputs();
    intake.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intake.periodic(); // jam now active

    // stopIntake must clear jam state so subsequent spinIntake works normally
    intake.stopIntake().initialize();
    Mockito.clearInvocations(mockIntakeIO);

    intake.spinIntake().initialize();

    // Now spinning should call setIntakeVelocity (not be blocked by jam state)
    verify(mockIntakeIO).setIntakeVelocity(any(AngularVelocity.class));
  }

  @Test
  void onJamDetected_callback_invoked() {
    AtomicBoolean callbackFired = new AtomicBoolean(false);
    intake = new Intake(mockIntakeIO, () -> callbackFired.set(true));

    intake.spinIntake().initialize();
    setupStallInputs();
    intake.periodic();
    SimHooks.stepTiming(IntakeConstants.STALL_DEBOUNCE_SECONDS + 0.02);
    intake.periodic();

    assertTrue(callbackFired.get(), "onJamDetected callback should have been invoked");
  }

  // ── helpers ───────────────────────────────────────────────────────────────

  private void setupStallInputs() {
    doAnswer(
            inv -> {
              IntakeIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              // Velocity well below the 5 RPS stall threshold
              inputs.intakeVelocity = RotationsPerSecond.of(1.0);
              // Current well above the 40A stall threshold
              inputs.intakeAppliedCurrent = Amps.of(60.0);
              return null;
            })
        .when(mockIntakeIO)
        .updateInputs(any(IntakeIOInputs.class));
  }

  private void setupRecoveredInputs() {
    doAnswer(
            inv -> {
              IntakeIOInputs inputs = inv.getArgument(0);
              inputs.intakeTalonConnected = true;
              // Velocity well above the 5 RPS stall threshold
              inputs.intakeVelocity = RotationsPerSecond.of(20.0);
              inputs.intakeAppliedCurrent = Amps.of(10.0);
              return null;
            })
        .when(mockIntakeIO)
        .updateInputs(any(IntakeIOInputs.class));
  }
}
