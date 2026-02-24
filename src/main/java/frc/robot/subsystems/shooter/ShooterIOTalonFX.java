package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public class ShooterIOTalonFX implements ShooterIO {
    // Here we have any "member" variables, which are variables that live
    // inside the class and can be used in any of the methods.
    private final TalonFX m_shooterL, m_shooterM, m_shooterR;
    private final ServoHub m_hub;

    // Control modes for closed loop control
    private final VelocityVoltage m_velVolt = new VelocityVoltage(0.0);
    private final VelocityTorqueCurrentFOC m_velTorque = new VelocityTorqueCurrentFOC(0.0);

    // Status signals are how we read data from the talons
    private final StatusSignal<AngularVelocity> m_velocityL;
    private final StatusSignal<AngularVelocity> m_velocityM;
    private final StatusSignal<AngularVelocity> m_velocityR;

    private final StatusSignal<Voltage> m_voltageL;
    private final StatusSignal<Voltage> m_voltageM;
    private final StatusSignal<Voltage> m_voltageR;

    private final StatusSignal<Current> m_statorCurrentL;
    private final StatusSignal<Current> m_statorCurrentM;
    private final StatusSignal<Current> m_statorCurrentR;

    private final StatusSignal<Current> m_supplyCurrentL;
    private final StatusSignal<Current> m_supplyCurrentM;
    private final StatusSignal<Current> m_supplyCurrentR;

    public ShooterIOTalonFX() {
        m_shooterL = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID, Constants.CANIVORE_BUS);
        m_shooterM = new TalonFX(ShooterConstants.SHOOTER_MIDDLE_ID, Constants.CANIVORE_BUS);
        m_shooterR = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID, Constants.CANIVORE_BUS);

        m_hub = new ServoHub(ShooterConstants.SERVO_HUB_ID);

        // Apply the configs to the motors
        m_shooterL.getConfigurator().apply(ShooterConstants.TALON_FX_CONFIGURATION);
        m_shooterM.getConfigurator().apply(ShooterConstants.TALON_FX_CONFIGURATION);
        m_shooterR.getConfigurator().apply(ShooterConstants.TALON_FX_CONFIGURATION
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)));

        // get all of our signals so we can recieve data
        m_velocityL = m_shooterL.getVelocity();
        m_velocityM = m_shooterM.getVelocity();
        m_velocityR = m_shooterR.getVelocity();

        m_voltageL = m_shooterL.getMotorVoltage();
        m_voltageM = m_shooterM.getMotorVoltage();
        m_voltageR = m_shooterR.getMotorVoltage();

        m_statorCurrentL = m_shooterL.getStatorCurrent();
        m_statorCurrentM = m_shooterM.getStatorCurrent();
        m_statorCurrentR = m_shooterR.getStatorCurrent();

        m_supplyCurrentL = m_shooterL.getSupplyCurrent();
        m_supplyCurrentM = m_shooterM.getSupplyCurrent();
        m_supplyCurrentR = m_shooterR.getSupplyCurrent();

        m_shooterL.optimizeBusUtilization();
        m_shooterM.optimizeBusUtilization();
        m_shooterR.optimizeBusUtilization();

        m_hub.configure(ShooterConstants.SERVO_HUB_CONFIG, ResetMode.kResetSafeParameters);

        m_hub.getServoChannel(ShooterConstants.LEFT_SERVO_CHANNEL).setEnabled(true);
        m_hub.getServoChannel(ShooterConstants.LEFT_SERVO_CHANNEL).setPowered(true);

        m_hub.getServoChannel(ShooterConstants.RIGHT_SERVO_CHANNEL).setEnabled(true);
        m_hub.getServoChannel(ShooterConstants.RIGHT_SERVO_CHANNEL).setPowered(true);

        m_hub.getServoChannel(ShooterConstants.LEFT_SERVO_CHANNEL)
                .setPulseWidth(getPulseWidth(0.1));
        m_hub.getServoChannel(ShooterConstants.RIGHT_SERVO_CHANNEL)
                .setPulseWidth(getPulseWidth(0.1));
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        // refresh all our signals so we get new datas
        BaseStatusSignal.refreshAll(
                m_velocityL,
                m_velocityM,
                m_velocityR,
                m_voltageL,
                m_voltageM,
                m_voltageR,
                m_statorCurrentL,
                m_statorCurrentM,
                m_statorCurrentR,
                m_supplyCurrentL,
                m_supplyCurrentM,
                m_supplyCurrentR
        );

        // update all our signals to log and use elsewhere
        inputs.shooterSpeedL = m_velocityL.getValue();
        inputs.shooterSpeedM = m_velocityM.getValue();
        inputs.shooterSpeedR = m_velocityR.getValue();

        inputs.shooterVoltageL = m_voltageL.getValue();
        inputs.shooterVoltageM = m_voltageM.getValue();
        inputs.shooterVoltageR = m_voltageR.getValue();

        inputs.shooterStatorCurrentL = m_statorCurrentL.getValue();
        inputs.shooterStatorCurrentM = m_statorCurrentM.getValue();
        inputs.shooterStatorCurrentR = m_statorCurrentR.getValue();

        inputs.shooterSupplyCurrentL = m_supplyCurrentL.getValue();
        inputs.shooterSupplyCurrentM = m_supplyCurrentM.getValue();
        inputs.shooterSupplyCurrentR = m_supplyCurrentR.getValue();
    }

    // generic "open loop" control, using the WPI Units library for voltage
    @Override
    public void setShooterVoltage(Voltage voltage) {
        m_shooterL.setVoltage(voltage.in(Volts));
        m_shooterM.setVoltage(voltage.in(Volts));
        m_shooterR.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setShooterRPM(AngularVelocity velocity) {
        m_shooterL.setControl(m_velVolt.withVelocity(velocity));
        m_shooterM.setControl(m_velVolt.withVelocity(velocity));
        m_shooterR.setControl(m_velVolt.withVelocity(velocity));
    }

    @Override
    public void setHoodDistance(double position) {
        m_hub.getServoChannel(ShooterConstants.LEFT_SERVO_CHANNEL)
                .setPulseWidth(getPulseWidth(position));
        m_hub.getServoChannel(ShooterConstants.RIGHT_SERVO_CHANNEL)
                .setPulseWidth(getPulseWidth(position));
    }

    // servo min is 1000 and max is 2000
    private int getPulseWidth(double percentage) {
        return (int) (1000 * percentage) + 1000;
    }
}
