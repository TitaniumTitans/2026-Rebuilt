package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double[] shooterSpeeds = {0.0, 0.0, 0.0};
        public double columnPower = 0;
        
    }
}
