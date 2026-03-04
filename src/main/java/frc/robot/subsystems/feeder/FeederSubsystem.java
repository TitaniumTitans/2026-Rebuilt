package frc.robot.subsystems.feeder;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Feeder", m_inputs);
    }

    public Command runFeeder(Voltage voltage) {
        return runEnd(() -> {
            m_io.setFeederVoltage(voltage);
            m_io.setColumnVoltage(voltage);
        }, () -> {
            m_io.setFeederVoltage(Volts.of(0.0));
            m_io.setColumnVoltage(Volts.of(0.0));
        });
    }
}

