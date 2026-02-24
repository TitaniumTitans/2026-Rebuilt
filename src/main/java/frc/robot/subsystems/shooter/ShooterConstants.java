package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

public class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 18;
    public static final int SHOOTER_MIDDLE_ID = 19;
    public static final int SHOOTER_RIGHT_ID = 20;
    public static final int SERVO_HUB_ID = 21;

    public static final ServoChannel.ChannelId LEFT_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId0;
    public static final ServoChannel.ChannelId RIGHT_SERVO_CHANNEL = ServoChannel.ChannelId.kChannelId1;

    public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(120)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast))
            .withTorqueCurrent(new TorqueCurrentConfigs());

    public static final ServoHubConfig SERVO_HUB_CONFIG = new ServoHubConfig();
    static {
        SERVO_HUB_CONFIG.channel0.pulseRange(1000, 1500, 2000);
        SERVO_HUB_CONFIG.channel0.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);

        SERVO_HUB_CONFIG.channel1.pulseRange(1000, 1500, 2000);
        SERVO_HUB_CONFIG.channel1.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
    }
}
