package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

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
        m_shooterL = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID);
        m_shooterM = new TalonFX(ShooterConstants.SHOOTER_MIDDLE_ID);
        m_shooterR = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID);

        m_hub = new ServoHub(ShooterConstants.SERVO_HUB_ID);

        // Apply the configs to the motors
        m_shooterL.getConfigurator().apply(ShooterConstants.cfg);
        m_shooterM.getConfigurator().apply(ShooterConstants.cfg);
        m_shooterR.getConfigurator().apply(ShooterConstants.cfg);

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
        inputs.shooterSpeeds = new double[]{
                m_velocityL.getValue().in(RPM),
                m_velocityM.getValue().in(RPM),
                m_velocityR.getValue().in(RPM)
        };

        inputs.shooterVoltages = new double[]{
                m_voltageL.getValue().in(Volts),
                m_voltageM.getValue().in(Volts),
                m_voltageR.getValue().in(Volts)
        };

        inputs.shooterStatorCurrent = new double[]{
                m_statorCurrentL.getValue().in(Amps),
                m_statorCurrentM.getValue().in(Amps),
                m_statorCurrentR.getValue().in(Amps)
        };

        inputs.shooterSupplyCurrent = new double[]{
                m_supplyCurrentL.getValue().in(Amps),
                m_supplyCurrentM.getValue().in(Amps),
                m_supplyCurrentR.getValue().in(Amps)
        };
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

    // servo min is 1000 and max is 1000
    private int getPulseWidth(double percentage) {
        return (int) (1000 * percentage) + 1000;
    }
}
