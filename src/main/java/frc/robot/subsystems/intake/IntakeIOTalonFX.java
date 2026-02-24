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
    private final StatusSignal<Current> pivotSupplyCurrent;
    private final StatusSignal<Current> pivotStatorCurrent;

    private final PidProperty m_pivotPID;

    public IntakeIOTalonFX() {
        m_pivot = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);
        m_intake = new TalonFX(IntakeConstants.INTAKE_ID);

        m_pivot.getConfigurator().apply(IntakeConstants.PIVOT_CONFIG);

        pivotAngle = m_pivot.getPosition();
        pivotSupplyCurrent = m_pivot.getSupplyCurrent();
        pivotStatorCurrent = m_pivot.getStatorCurrent();

        m_pivotPID = new Phoenix6TalonPidPropertyBuilder("Intake Pivot", false, m_pivot, 0)
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
                pivotSupplyCurrent,
                pivotStatorCurrent
        );

        inputs.pivotAngleDegrees = pivotAngle.getValue().in(Degree);
        inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValue().in(Amps);
        inputs.pivotStatorCurrent = pivotStatorCurrent.getValue().in(Amps);

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
