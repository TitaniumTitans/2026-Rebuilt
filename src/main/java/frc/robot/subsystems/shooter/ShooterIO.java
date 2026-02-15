package frc.robot.subsystems.shooter;

import edu.wpi.first.units.AngularVelocityUnit;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double[] shooterSpeeds = {0.0, 0.0, 0.0};
        public double[] shooterStatorCurrent = {0.0, 0.0, 0.0};
        public double[] shooterSupplyCurrent = {0.0, 0.0, 0.0};
        public double feederPower = 0;
        public double feederStatorCurrent = 0;
        public double feederSupplyCurrent = 0;
        public double[] hoodPosition = {0.0, 0.0};
    }

    default void setShooterRPM(AngularVelocity velocity) {};
    default void setHoodDistance(Distance position) {};
    default void setFeederVoltage(Voltage voltage) {};
}
