package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedNetworkPIDFeedforwardGains {
  private LoggedNetworkNumber kP;
  private LoggedNetworkNumber kI;
  private LoggedNetworkNumber kD;
  private LoggedNetworkNumber kA;
  private LoggedNetworkNumber kV;
  private LoggedNetworkNumber kS;
  private LoggedNetworkNumber kG;

  public LoggedNetworkPIDFeedforwardGains(
      double defaultKP,
      double defaultKI,
      double defaultKD,
      double defaultKA,
      double defaultKV,
      double defaultKS,
      double defaultKG,
      String namePrefix) {
    kP = new LoggedNetworkNumber("Gains/" + namePrefix + "/kP", defaultKP);
    kI = new LoggedNetworkNumber("Gains/" + namePrefix + "/kI", defaultKI);
    kD = new LoggedNetworkNumber("Gains/" + namePrefix + "/kD", defaultKD);
    kA = new LoggedNetworkNumber("Gains/" + namePrefix + "/kA", defaultKA);
    kV = new LoggedNetworkNumber("Gains/" + namePrefix + "/kV", defaultKV);
    kS = new LoggedNetworkNumber("Gains/" + namePrefix + "/kS", defaultKS);
    kG = new LoggedNetworkNumber("Gains/" + namePrefix + "/kG", defaultKG);
  }

  public Slot0Configs toSlot0Configs() {
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP.get();
    configs.kI = kI.get();
    configs.kD = kD.get();
    configs.kA = kA.get();
    configs.kV = kV.get();
    configs.kS = kS.get();
    configs.kG = kG.get();
    return configs;
  }
}
