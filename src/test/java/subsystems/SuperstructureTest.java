package subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.CurrentState;
import frc.robot.subsystems.superstructure.Superstructure.WantedState;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class SuperstructureTest {
  Superstructure superstructure;
  Shooter mockShooter;
  Hopper mockHopper;
  IntakePivot mockIntake;
  Drive mockDrive;
  SwerveDriveSimulation mockDriveSimulation;

  @BeforeEach
  @SuppressWarnings("unchecked")
  void setup() {
    mockShooter = Mockito.mock(Shooter.class);
    mockHopper = Mockito.mock(Hopper.class);
    mockIntake = Mockito.mock(IntakePivot.class);
    mockDrive = Mockito.mock(Drive.class);
    mockDriveSimulation = Mockito.mock(SwerveDriveSimulation.class);

    // Stub command-returning methods so CommandScheduler.schedule() never gets null
    when(mockHopper.stopFullIndexer()).thenReturn(Commands.none());
    when(mockHopper.stopIndexer()).thenReturn(Commands.none());
    when(mockHopper.spinIndexer()).thenReturn(Commands.none());
    when(mockHopper.oscillateTopIndexer()).thenReturn(Commands.none());
    when(mockShooter.setFlywheelVoltage(any(Supplier.class))).thenReturn(Commands.none());
    when(mockShooter.setFeederVoltage(any(Supplier.class))).thenReturn(Commands.none());
    when(mockShooter.setFeederSpeed(any(Supplier.class))).thenReturn(Commands.none());
    when(mockShooter.setFlywheelRPM(any(Supplier.class))).thenReturn(Commands.none());
    when(mockIntake.setIntakeAngle(any(Supplier.class))).thenReturn(Commands.none());

    // Default: feeder up to speed (tests that need it false can override per-test)
    when(mockShooter.feederUpToSpeed()).thenReturn(true);

    // Default: drive pose/rotation at origin — within the 3.5° hub tolerance
    when(mockDrive.getPose()).thenReturn(new Pose2d());
    when(mockDrive.getRotation()).thenReturn(Rotation2d.kZero);
    when(mockDrive.getChassisSpeeds()).thenReturn(new ChassisSpeeds());

    superstructure =
        new Superstructure(mockDrive, mockShooter, mockHopper, mockIntake, mockDriveSimulation);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    superstructure = null;
  }

  // ── handleWantedState() / setWantedSuperState() ─────────────────────────────

  @Test
  void idle_wantedState_setsCurrentStateIdle() {
    superstructure.setWantedSuperState(WantedState.IDLE);
    assertEquals(CurrentState.IDLE, superstructure.currentSuperState);
  }

  @Test
  void shooting_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyShooting() {
    // Start from IDLE
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.setWantedSuperState(WantedState.SHOOTING);
    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
  }

  @Test
  void shooting_wantedState_staysInShooting_whenAlreadyShooting() {
    // Already SHOOTING — handleWantedState should not revert to SPINNING_UP
    superstructure.currentSuperState = CurrentState.SHOOTING;
    superstructure.setWantedSuperState(WantedState.SHOOTING);
    assertEquals(CurrentState.SHOOTING, superstructure.currentSuperState);
  }

  @Test
  void passing_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyPassing() {
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.setWantedSuperState(WantedState.PASSING);
    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
  }

  @Test
  void passing_wantedState_staysInPassing_whenAlreadyPassing() {
    superstructure.currentSuperState = CurrentState.PASSING;
    superstructure.setWantedSuperState(WantedState.PASSING);
    assertEquals(CurrentState.PASSING, superstructure.currentSuperState);
  }

  // ── readyToShoot() ───────────────────────────────────────────────────────────

  @Test
  void readyToShoot_returnsTrue_whenFlywheelAtSpeedAndRotationInTolerance() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    // drive.getRotation() = 0° (from setup), shotSetpoint.robotPose default rotation = 0° → 0°
    // error
    assertTrue(superstructure.readyToShoot());
  }

  @Test
  void readyToShoot_returnsFalse_whenFlywheelNotAtSpeed() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(false);
    assertFalse(superstructure.readyToShoot());
  }

  @Test
  void readyToShoot_returnsFalse_whenRotationOutOfTolerance() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    // 10° error — exceeds the 3.5° HUB_ROTATION_TOLERANCE
    when(mockDrive.getRotation()).thenReturn(Rotation2d.fromDegrees(10));
    assertFalse(superstructure.readyToShoot());
  }

  // ── applyStates() transitions ─────────────────────────────────────────────

  @Test
  void Spinning_Up_To_Shooting_Transition() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    // feederUpToSpeed() defaults to true in @BeforeEach
    // drive.getRotation() = 0° (within tolerance) — already set in setup

    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.currentSuperState = CurrentState.SPINNING_UP;

    superstructure.applyStates();

    // SPINNING_UP → REVERSING (brief feeder/indexer reversal) → SHOOTING
    assertEquals(CurrentState.REVERSING, superstructure.currentSuperState);
  }

  @Test
  void Spinning_Up_To_Passing_Transition() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);

    superstructure.wantedSuperState = WantedState.PASSING;
    superstructure.currentSuperState = CurrentState.SPINNING_UP;

    superstructure.applyStates();

    assertEquals(CurrentState.PASSING, superstructure.currentSuperState);
  }

  @Test
  void spinningUp_notReady_staysSpinningUp() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(false);

    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.currentSuperState = CurrentState.SPINNING_UP;

    superstructure.applyStates();

    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
  }

  @Test
  void spinningUp_shootingButRotationOutOfTolerance_staysSpinningUp() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    when(mockDrive.getRotation()).thenReturn(Rotation2d.fromDegrees(10)); // out of tolerance

    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.currentSuperState = CurrentState.SPINNING_UP;

    superstructure.applyStates();

    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
  }

  @Test
  void idleState_schedulesStopCommands() {
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.applyStates();

    // Verify that stop commands were requested from the subsystems
    verify(mockHopper).stopFullIndexer();
    verify(mockShooter).setFlywheelVoltage(any(Supplier.class));
    verify(mockShooter).setFeederVoltage(any(Supplier.class));
  }
}
