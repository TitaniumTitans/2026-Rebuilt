package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class FeederConstants {
    public static final int FEEDER_ID = 16;
    public static final int COLUMN_ID = 17;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true));
}
