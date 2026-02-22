package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class ShooterIOTalonFX implements ShooterIO {
    // Here we have any "member" variables, which are variables that live
    // inside the class and can be used in any of the methods.
    private final TalonFX m_shooterL, m_shooterM, m_shooterR;
    private final ServoHub m_hub;


    public ShooterIOTalonFX() {
        m_shooterL = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID);
        m_shooterM = new TalonFX(ShooterConstants.SHOOTER_MIDDLE_ID);
        m_shooterR = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID);

        m_hub = new ServoHub(ShooterConstants.SERVO_HUB_ID);

        m_shooterL.getConfigurator().apply(ShooterConstants.cfg);
        m_shooterM.getConfigurator().apply(ShooterConstants.cfg);
        m_shooterR.getConfigurator().apply(ShooterConstants.cfg);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {

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

    }

    @Override
    public void setHoodDistance(Distance position) {
    }
}
