package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
    // All the data from the subsystem we want to either use elsewhere
    // in robot code or log gets put here
    @AutoLog
    class ShooterIOInputs {
        public AngularVelocity shooterSpeedL, shooterSpeedM, shooterSpeedR = RPM.of(0.0);
        public Voltage shooterVoltageL, shooterVoltageM, shooterVoltageR = Volts.of(0.0);
        public Current shooterStatorCurrentL, shooterStatorCurrentM, shooterStatorCurrentR = Amps.of(0.0);
        public Current shooterSupplyCurrentL, shooterSupplyCurrentM, shooterSupplyCurrentR = Amps.of(0.0);

        public double[] hoodPositionPercent = {0.0, 0.0};
    }

    // Required method to update all the logged inputs
    default void updateInputs(ShooterIOInputsAutoLogged inputs) {};

    // Any hardware output, usually motor voltage and PID control
    default void setShooterVoltage(Voltage voltage) {};
    default void setShooterRPM(AngularVelocity velocity) {};
    default void setHoodDistance(double position) {};
}
