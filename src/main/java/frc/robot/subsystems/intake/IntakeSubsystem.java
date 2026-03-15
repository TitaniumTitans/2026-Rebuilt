package frc.robot.subsystems.intake;


import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private static final GosDoubleProperty m_intakeMultiplier =
            new GosDoubleProperty(false, "Intake/Intake Speed Multiplier", 0.55);

    public enum Speed {
        STOP(0),
        INTAKE(1.0); // 0.55

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0 * m_intakeMultiplier.getValue());
        }
    }

    public enum Position {
        HOMED(new GosDoubleProperty(false, "Intake/Homed Angle", 130)),
        STOWED(new GosDoubleProperty(false, "Intake/Stowed Angle", 120)),
        INTAKE(new GosDoubleProperty(false, "Intake/Intake Angle", 0)),
        AGITATE(new GosDoubleProperty(false, "Intake/Agitate Angle", 20));

        private final GosDoubleProperty degrees;

        private Position(GosDoubleProperty degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees.getValue());
        }
    }

    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

    private boolean isHomed = false;

    public IntakeSubsystem(IntakeIO io) {
        m_io = io;
        m_io.resetPivotAngle(Position.HOMED.angle());
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

    public Command setIntakePower(Speed speed) {
        return runOnce(() -> m_io.setIntakeVoltage(speed.voltage()));
    }

    // runs the standard intake procedure
    public Command intake() {
        return startEnd(
                () -> {
                    m_io.setPivotAngle(Position.INTAKE.angle());
                    m_io.setIntakeVoltage(Speed.INTAKE.voltage());
                },
                () -> {
                    m_io.setIntakeVoltage(Speed.STOP.voltage());
                    m_io.setPivotAngle(Position.AGITATE.angle());
                }
        );
    }

    // runs the intake back until stall to home without a limit switch
    public Command homingCommand() {
        return Commands.sequence(
                        runOnce(() -> m_io.setPivotVoltage(Volts.of(1.2))),
                        Commands.waitUntil(() -> m_inputs.pivotSupplyCurrent.in(Amps) > 6),
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
