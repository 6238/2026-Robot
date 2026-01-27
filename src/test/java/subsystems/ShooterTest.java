package subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class ShooterTest {
  static final double DELTA = 1e-2;

  ShooterIO mockShooterIO;
  Shooter shooter;

  @BeforeEach
  void setup() {
    mockShooterIO = Mockito.mock(ShooterIO.class);
    shooter = new Shooter(mockShooterIO);
  }

  @Test
  void Positive_Up_To_Speed_Check() {
    AngularVelocity targetVelocity = RotationsPerSecond.of(90);

    // set target velocity
    shooter.setFlywheelRPM(targetVelocity);

    // change update inputs to set new test velocity
    Mockito.doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = targetVelocity;
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(Mockito.any(ShooterIOInputs.class));

    // call periodic on shooter to update the inputs
    shooter.periodic();

    // check shooter up to speed function
    Assertions.assertTrue(shooter.flywheelUpToSpeed());
  }

  @Test
  void Negative_Up_To_Speed_Check() {
    AngularVelocity targetVelocity = RotationsPerSecond.of(90);
    AngularVelocity incorrectVelocity = RotationsPerSecond.of(-1);

    // set target velocity
    shooter.setFlywheelRPM(targetVelocity);

    // change update inputs to set new test velocity
    Mockito.doAnswer(
            inv -> {
              ShooterIOInputs inputs = inv.getArgument(0);
              inputs.flywheelTalonConnected = true;
              inputs.feederTalonConnected = true;
              inputs.flywheelVelocity = incorrectVelocity;
              return null;
            })
        .when(mockShooterIO)
        .updateInputs(Mockito.any(ShooterIOInputs.class));

    // call periodic on shooter to update the inputs
    shooter.periodic();

    // check shooter up to speed function
    Assertions.assertFalse(shooter.flywheelUpToSpeed());
  }

  @AfterEach
  void tearDown() {
    shooter = null;
  }
}
