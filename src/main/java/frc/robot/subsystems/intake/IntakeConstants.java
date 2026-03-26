package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class IntakeConstants {
    public static final int INTAKE_PIVOT_ID = 15;
    public static final int INTAKE_ID = 14;

    public static final double PIVOT_GEAR_RATIO = 50;
    // 6000 RPM
    public static final AngularVelocity PIVOT_MAX_SPEED = RPM.of(5800).div(PIVOT_GEAR_RATIO);

    public static final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(PIVOT_MAX_SPEED)
                    .withMotionMagicAcceleration(PIVOT_MAX_SPEED.per(Second)))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(PIVOT_GEAR_RATIO)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLimitEnable(true)
            );

    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(60))
                .withSupplyCurrentLimitEnable(true)
        )
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)

        );

}
