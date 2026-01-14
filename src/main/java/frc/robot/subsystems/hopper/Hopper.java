package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  public HopperIO io;
  public HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO io) {
    this.io = io;
    this.inputs = new HopperIOInputsAutoLogged();
  }
}
