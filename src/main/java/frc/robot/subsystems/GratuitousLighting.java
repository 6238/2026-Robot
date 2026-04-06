package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GratuitousLighting extends SubsystemBase {
  private CANBus candleCanBus = new CANBus("canivore");
  private CANdle candle;

  private ControlRequest intakeControlRequest = new FireAnimation(0, 41);

  public GratuitousLighting() {
    candle = new CANdle(45, candleCanBus);
  }

  public Command intaking() {
    return runOnce(
        () -> {
          candle.setControl(intakeControlRequest);
        });
  }
}
