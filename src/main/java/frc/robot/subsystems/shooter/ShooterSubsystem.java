package frc.robot.subsystems.shooter;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volt;

public class ShooterSubsystem extends SubsystemBase {
    // IO layer handles all talking to real hardware
    private final ShooterIO m_io;

    // IO inputs class handles getting data from hardware
    private final ShooterIOInputsAutoLogged m_inputs;

    public ShooterSubsystem(ShooterIO io) {
        m_io = io;
        m_inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // update all our input values from our hardware/sim
        m_io.updateInputs(m_inputs);
        Logger.processInputs("shooter", m_inputs);
    }

    // Commands are what get bound to buttons
    // They use lambas, essentially using a function as data
    public Command setShooterVoltage(Voltage voltage) {
        return runEnd(
                // Start conditions
                () -> m_io.setShooterVoltage(voltage),
                // End conditions
                () -> m_io.setShooterVoltage(Volt.of(0.0))
        );
    }
}

