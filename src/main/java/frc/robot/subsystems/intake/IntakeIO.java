package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double intakeVoltage = 0.0;
        public double intakeStatorCurrent = 0.0;
        public double intakeSupplyCurrent = 0.0;

        public double pivotAngleDegrees = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotStatorCurrent = 0.0;
        public double pivotSupplyCurrent = 0.0;

        public double feederVoltage = 0.0;
        public double feederStatorCurrent = 0.0;
        public double feederSupplyCurrent = 0.0;
    }

    default void setIntakeVoltage(Voltage voltage) {};
    default void setPivotVoltage(Voltage voltage) {};
    default void setPivotAngle(Angle angle) {};
    default void setFeederVoltage(Voltage voltage) {};
}
