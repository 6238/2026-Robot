package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.util.AlertUtils;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.measure.Angle;

public class Climber extends SubsystemBase {
    private TalonFX motorController;
    public Alert ClimberMotorConnectedAlert =
            new Alert("Critical", "Climber Motor Disconnected", AlertType.kError);

    public ClimberIO io;
    public ClimberIOInputsAutoLogged inputs;

    public Climber(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberIOInputsAutoLogged();
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motorController.getConfigurator().apply(talonFXConfigs);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        AlertUtils.processCriticalAlert(climberMotorConnectedAlert, !inputs.climberTalonConnected);
    }

    public Command setPosition(double position) {
        return runOnce(
            () -> {
                // create a Motion Magic request, voltage output
                final MotionMagicVoltage request = new MotionMagicVoltage(0);
                motorController.setControl(request.withPosition(position));
            });
    }
    public StatusSignal<Angle> getPosition() {
        return motorController.getPosition();
    }
}
