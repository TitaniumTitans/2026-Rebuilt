package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX m_pivot, m_intake;

    private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0.0);

    private final StatusSignal<Angle> pivotAngle;
    private final StatusSignal<Voltage> pivotVoltage, intakeVoltage;
    private final StatusSignal<Current> pivotSupplyCurrent, pivotStatorCurrent,
            intakeSupplyCurrent, intakeStatorCurrent;

    private final PidProperty m_pivotPID;

    public IntakeIOTalonFX() {
        m_pivot = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);
        m_intake = new TalonFX(IntakeConstants.INTAKE_ID);

        m_pivot.getConfigurator().apply(IntakeConstants.PIVOT_CONFIG);

        // create signals for pivot motor
        pivotAngle = m_pivot.getPosition();
        pivotVoltage = m_pivot.getMotorVoltage();
        pivotSupplyCurrent = m_pivot.getSupplyCurrent();
        pivotStatorCurrent = m_pivot.getStatorCurrent();

        // create signals for intake motor
        intakeVoltage = m_intake.getMotorVoltage();
        intakeSupplyCurrent = m_intake.getSupplyCurrent();
        intakeStatorCurrent = m_intake.getStatorCurrent();

        m_pivotPID = new Phoenix6TalonPidPropertyBuilder("Intake/Pivot/", false, m_pivot, 0)
                .addP(10)
                .addI(0)
                .addD(0)
                .addKV(0)
                .build();
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
                pivotAngle,
                pivotVoltage,
                pivotSupplyCurrent,
                pivotStatorCurrent,
                intakeVoltage,
                intakeSupplyCurrent,
                intakeStatorCurrent
        );

        inputs.pivotAngleDegrees = pivotAngle.getValue();
        inputs.pivotVoltage = pivotVoltage.getValue();
        inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValue();
        inputs.pivotStatorCurrent = pivotStatorCurrent.getValue();

        inputs.intakeVoltage = intakeVoltage.getValue();
        inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();
        inputs.intakeStatorCurrent = intakeStatorCurrent.getValue();

        m_pivotPID.updateIfChanged();
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        m_pivot.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setPivotAngle(Angle angle) {
        m_pivot.setControl(m_mmVoltage.withPosition(angle));
    }

    @Override
    public void setIntakeVoltage(Voltage voltage) {
        m_intake.setVoltage(voltage.in(Volts));
    }

    @Override
    public void resetPivotAngle(Angle angle) {
        m_pivot.setPosition(angle);
    }
}
