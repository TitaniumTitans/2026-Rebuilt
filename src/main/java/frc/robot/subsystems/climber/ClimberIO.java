package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double climberPosition = 0.0;
        public double climberVoltage = 0.0;
        public double climberSupplyCurrent = 0.0;
        public double climberStatorCurrent = 0.0;
    }

    default void setClimberVoltage(Voltage voltage) {};
    default void setClimberPosition(Distance position) {};
}
