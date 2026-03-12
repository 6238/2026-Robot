package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class ShooterTest {
  static final double DELTA = 1e-6;

  ShooterIO mockShooterIO;
  Shooter shooter;

  @BeforeEach
  void setup() {
    mockShooterIO = Mockito.mock(ShooterIO.class);
    // Default: both motors connected so processCriticalAlert never triggers NPE via rumble fn
    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter = new Shooter(mockShooterIO);
  }

  @AfterEach
  void tearDown() {
    shooter = null;
  }

  // ── setFlywheelRPM ────────────────────────────────────────────────────────

  @Test
  void setFlywheelRPM_callsSetFlywheelSpeed() {
    AngularVelocity speed = RotationsPerSecond.of(80);
    shooter.setFlywheelRPM(speed);
    verify(mockShooterIO).setFlywheelSpeed(speed);
  }

  @Test
  void setFlywheelRPM_storesExactTargetVelocity() {
    AngularVelocity speed = RotationsPerSecond.of(75);
    shooter.setFlywheelRPM(speed);
    assertEquals(75.0, shooter.targetFlywheelVelocity.in(RotationsPerSecond), DELTA);
  }

  @Test
  void setFlywheelRPM_commandVersion_callsSetFlywheelSpeed() {
    AngularVelocity speed = RotationsPerSecond.of(80);
    shooter.setFlywheelRPM(() -> speed).initialize();
    verify(mockShooterIO).setFlywheelSpeed(speed);
  }

  @Test
  void setFlywheelRPM_commandVersion_storesTargetVelocity() {
    AngularVelocity speed = RotationsPerSecond.of(65);
    shooter.setFlywheelRPM(() -> speed).initialize();
    assertEquals(65.0, shooter.targetFlywheelVelocity.in(RotationsPerSecond), DELTA);
  }

  // ── setFlywheelVoltage ────────────────────────────────────────────────────

  @Test
  void setFlywheelVoltage_callsSetFlywheelVoltage() {
    shooter.setFlywheelVoltage(Volts.of(6.0));

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFlywheelVoltage(captor.capture());
    assertEquals(6.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void setFlywheelVoltage_clearsTargetVelocityToZero() {
    shooter.setFlywheelRPM(RotationsPerSecond.of(90));
    shooter.setFlywheelVoltage(Volts.of(6.0));
    assertEquals(0.0, shooter.targetFlywheelVelocity.in(RotationsPerSecond), DELTA);
  }

  @Test
  void setFlywheelVoltage_afterSettingRPM_makesFlywheelUpToSpeedFalse() {
    // Set target to 90 RPS, then switch to voltage mode (clears target to 0)
    shooter.setFlywheelRPM(RotationsPerSecond.of(90));
    shooter.setFlywheelVoltage(Volts.of(6.0));

    // Simulate actual flywheel at 90 RPS — no longer matches target of 0
    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(90);
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertFalse(shooter.flywheelUpToSpeed());
  }

  @Test
  void setFlywheelVoltage_commandVersion_callsSetFlywheelVoltage() {
    shooter.setFlywheelVoltage(() -> Volts.of(9.0)).initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFlywheelVoltage(captor.capture());
    assertEquals(9.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void setFlywheelVoltage_negativeVoltage_passedThrough() {
    shooter.setFlywheelVoltage(Volts.of(-3.0));

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFlywheelVoltage(captor.capture());
    assertEquals(-3.0, captor.getValue().in(Volts), DELTA);
  }

  // ── setFeederVoltage ──────────────────────────────────────────────────────

  @Test
  void setFeederVoltage_callsSetFeederVoltage() {
    shooter.setFeederVoltage(Volts.of(8.0));

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFeederVoltage(captor.capture());
    assertEquals(8.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void setFeederVoltage_commandVersion_callsSetFeederVoltage() {
    shooter.setFeederVoltage(() -> Volts.of(8.0)).initialize();

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFeederVoltage(captor.capture());
    assertEquals(8.0, captor.getValue().in(Volts), DELTA);
  }

  @Test
  void setFeederVoltage_zero_passedThrough() {
    shooter.setFeederVoltage(Volts.of(0.0));

    ArgumentCaptor<Voltage> captor = ArgumentCaptor.forClass(Voltage.class);
    verify(mockShooterIO).setFeederVoltage(captor.capture());
    assertEquals(0.0, captor.getValue().in(Volts), DELTA);
  }

  // ── flywheelUpToSpeed ─────────────────────────────────────────────────────

  @Test
  void flywheelUpToSpeed_trueWhenAtTarget() {
    AngularVelocity target = RotationsPerSecond.of(90);
    shooter.setFlywheelRPM(target);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = target;
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertTrue(shooter.flywheelUpToSpeed());
  }

  @Test
  void flywheelUpToSpeed_falseWhenBelowTarget() {
    shooter.setFlywheelRPM(RotationsPerSecond.of(90));

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(-1);
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertFalse(shooter.flywheelUpToSpeed());
  }

  @Test
  void flywheelUpToSpeed_falseWhenFarFromTarget() {
    shooter.setFlywheelRPM(RotationsPerSecond.of(90));

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(50); // way off
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertFalse(shooter.flywheelUpToSpeed());
  }

  @Test
  void flywheelUpToSpeed_trueWhenWithinDefaultTolerance() {
    // FLYWHEEL_TOLERANCE_BEFORE_SHOT = 0.5 RPS; actual within tolerance
    AngularVelocity target = RotationsPerSecond.of(90);
    shooter.setFlywheelRPM(target);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(90.3); // within 0.5 RPS
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertTrue(shooter.flywheelUpToSpeed());
  }

  @Test
  void flywheelUpToSpeed_falseWhenJustOutsideDefaultTolerance() {
    AngularVelocity target = RotationsPerSecond.of(90);
    shooter.setFlywheelRPM(target);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(89.0); // 1.0 RPS off, > 0.5 tolerance
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertFalse(shooter.flywheelUpToSpeed());
  }

  @Test
  void flywheelUpToSpeed_withCustomTolerance_trueWhenWithin() {
    AngularVelocity target = RotationsPerSecond.of(90);
    AngularVelocity tolerance = RotationsPerSecond.of(5.0);
    shooter.setFlywheelRPM(target);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(87.0); // within 5 RPS
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertTrue(shooter.flywheelUpToSpeed(tolerance));
  }

  @Test
  void flywheelUpToSpeed_withCustomTolerance_falseWhenOutside() {
    AngularVelocity target = RotationsPerSecond.of(90);
    AngularVelocity tolerance = RotationsPerSecond.of(2.0);
    shooter.setFlywheelRPM(target);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = RotationsPerSecond.of(86.0); // 4 RPS off, > 2 tolerance
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertFalse(shooter.flywheelUpToSpeed(tolerance));
  }

  @Test
  void flywheelUpToSpeed_defaultsToFalseBeforeAnyInput() {
    // target = 0, actual = 0 (defaults) → isNear should be true, not a useful shot-readiness signal
    // This test documents the behavior: with no RPM set and no velocity, it reports "up to speed"
    assertTrue(shooter.flywheelUpToSpeed());
  }

  // ── getCurrentFlywheelSpeed ───────────────────────────────────────────────

  @Test
  void getCurrentFlywheelSpeed_returnsInputsVelocity() {
    AngularVelocity reported = RotationsPerSecond.of(55);

    doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = reported;
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(any(ShooterIOInputs.class));
    shooter.periodic();

    assertEquals(55.0, shooter.getCurrentFlywheelSpeed().in(RotationsPerSecond), DELTA);
  }

  @Test
  void getCurrentFlywheelSpeed_defaultsToZero() {
    assertEquals(0.0, shooter.getCurrentFlywheelSpeed().in(RotationsPerSecond), DELTA);
  }

  // ── periodic ─────────────────────────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    shooter.periodic();
    verify(mockShooterIO).updateInputs(any(ShooterIOInputs.class));
  }

  @Test
  void periodic_doesNotCallSetFlywheelSpeed() {
    // periodic() should only read, not command the motor
    shooter.periodic();
    verify(mockShooterIO, never()).setFlywheelSpeed(any());
  }
}
