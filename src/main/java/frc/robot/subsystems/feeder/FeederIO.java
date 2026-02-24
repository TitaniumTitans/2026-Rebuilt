package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public Voltage feederVoltage = Volts.of(0.0);
        public Current feederStatorCurrent = Amps.of(0.0);
        public Current feederSupplyCurrent = Amps.of(0.0);

        public Voltage columnVoltage = Volts.of(0.0);
        public Current columnStatorCurrent = Amps.of(0.0);
        public Current columnSupplyCurrent = Amps.of(0.0);
    }

    default void updateInputs(FeederIOInputs inputs) {}
    default void setFeederVoltage(Voltage voltage) {}
    default void setColumnVoltage(Voltage voltage) {}
}
