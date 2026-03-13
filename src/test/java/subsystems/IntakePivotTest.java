package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIO.IntakePivotIOInputs;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class IntakePivotTest {
  static final double DELTA = 1e-6;

  IntakePivotIO mockIntakePivotIO;
  IntakePivot intakePivot;

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
    mockIntakePivotIO = Mockito.mock(IntakePivotIO.class);
    doAnswer(
            inv -> {
              IntakePivotIOInputs inputs = inv.getArgument(0);
              inputs.intakeArmTalonConnected = true;
              return null;
            })
        .when(mockIntakePivotIO)
        .updateInputs(any(IntakePivotIOInputs.class));
    intakePivot = new IntakePivot(mockIntakePivotIO);
  }

  @AfterEach
  void tearDown() {
    intakePivot = null;
  }

  // ── setIntakeAngle ────────────────────────────────────────────────────────

  @Test
  void setIntakeAngle_callsSetIntakePosition() {
    intakePivot.setIntakeAngle(() -> Degrees.of(45.0)).initialize();
    verify(mockIntakePivotIO).setIntakePosition(any(Angle.class));
  }

  @Test
  void setIntakeAngle_setsCorrectAngle() {
    Angle targetAngle = Degrees.of(45.0);
    intakePivot.setIntakeAngle(() -> targetAngle).initialize();

    ArgumentCaptor<Angle> captor = ArgumentCaptor.forClass(Angle.class);
    verify(mockIntakePivotIO).setIntakePosition(captor.capture());
    assertEquals(45.0, captor.getValue().in(Degrees), DELTA);
  }

  @Test
  void setIntakeAngle_updatesTargetAngleField() {
    Angle targetAngle = Degrees.of(-35.0);
    intakePivot.setIntakeAngle(() -> targetAngle).initialize();
    assertEquals(-35.0, intakePivot.targetAngle.in(Degrees), DELTA);
  }

  // ── periodic ─────────────────────────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    intakePivot.periodic();
    verify(mockIntakePivotIO).updateInputs(any(IntakePivotIOInputs.class));
  }

  // ── reset ─────────────────────────────────────────────────────────────────

  @Test
  void reset_callsResetArmAngle() {
    intakePivot.reset().initialize();
    verify(mockIntakePivotIO).resetArmAngle();
  }
}
