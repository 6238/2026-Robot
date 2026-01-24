package subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.WantedState;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class SuperstructureTest {
  Superstructure superstructure;
  Shooter mockShooter;
  Hopper mockHopper;
  Drive mockDrive;
  SwerveDriveSimulation mockDriveSimulation;

  @BeforeEach
  void setup() {
    mockShooter = Mockito.mock(Shooter.class);
    mockHopper = Mockito.mock(Hopper.class);
    mockDrive = Mockito.mock(Drive.class);
    mockDriveSimulation = Mockito.mock(SwerveDriveSimulation.class);

    superstructure = new Superstructure(mockDrive, mockShooter, mockHopper, mockDriveSimulation);
  }

  @Test
  void Spinning_Up_To_Shooting_Transition() {
    boolean upToSpeed = false;
    Pose2d fakeSwervePose = new Pose2d();

    Mockito.when(mockShooter.flywheelUpToSpeed()).thenReturn(upToSpeed);

    superstructure.setWantedSuperState(WantedState.SHOOTING);
    superstructure.periodic();
  }

  @Test
  void Spinning_Up_To_Passing_Transition() {}

  @AfterEach
  void tearDown() {}
}
