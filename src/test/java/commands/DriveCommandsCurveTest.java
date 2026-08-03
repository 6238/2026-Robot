package commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.DriveCommands;
import org.junit.jupiter.api.Test;

class DriveCommandsCurveTest {

  @Test
  void curve_atZero_returnsZero() {
    assertEquals(0.0, DriveCommands.applyCurve(0.0), 1e-9);
  }

  @Test
  void curve_atFullDeflection_returnsOne() {
    assertEquals(1.0, DriveCommands.applyCurve(1.0), 1e-9);
  }

  @Test
  void curve_atHalfDeflection_greaterThanHalf() {
    double result = DriveCommands.applyCurve(0.5);
    assertTrue(result > 0.5, "Expected curve output > 0.5 at stick=0.5, got " + result);
    assertTrue(result < 1.0, "Expected curve output < 1.0 at stick=0.5");
  }

  @Test
  void curve_monotonicallyIncreasing() {
    double prev = 0.0;
    for (double x = 0.1; x <= 1.0; x += 0.1) {
      double curr = DriveCommands.applyCurve(x);
      assertTrue(curr > prev, "Curve not monotonically increasing at x=" + x);
      prev = curr;
    }
  }
}
