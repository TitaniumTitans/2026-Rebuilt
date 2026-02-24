package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public Voltage intakeVoltage = Volts.of(0.0);
        public Current intakeStatorCurrent = Amps.of(0);
        public Current intakeSupplyCurrent = Amps.of(0);

        public Angle pivotAngleDegrees = Degrees.of(0.0);
        public Voltage pivotVoltage = Volts.of(0);
        public Current pivotStatorCurrent = Amps.of(0);
        public Current pivotSupplyCurrent = Amps.of(0);
    }

    default void updateInputs(IntakeIOInputsAutoLogged inputs) {};
    default void setIntakeVoltage(Voltage voltage) {};
    default void setPivotVoltage(Voltage voltage) {};
    default void setPivotAngle(Angle angle) {};
    default void resetPivotAngle(Angle angle) {};
}
