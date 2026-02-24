package frc.robot.subsystems.intake;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

    private boolean isHomed = false;

    public IntakeSubsystem(IntakeIO io) {
        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Intake", m_inputs);
    }

    public Command setPivotVoltage(Voltage volts) {
        return runEnd(
                () -> m_io.setPivotVoltage(volts),
                () -> m_io.setPivotVoltage(Volts.of(0.0))
        );
    }

    public Command setPivotPosition(Position position) {
        return runOnce(() -> m_io.setPivotAngle(position.angle()));
    }

    public Command homingCommand() {
        return Commands.sequence(
                        runOnce(() -> m_io.setPivotVoltage(Volts.of(1.2))),
                        Commands.waitUntil(() -> m_inputs.pivotSupplyCurrent > 6),
                        runOnce(() -> {
                            m_io.resetPivotAngle(Position.HOMED.angle());
                            isHomed = true;
                        }),
                        runOnce(() -> m_io.setPivotVoltage(Volts.of(0.0)))
                )
//                .unless(() -> isHomed)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }
}
