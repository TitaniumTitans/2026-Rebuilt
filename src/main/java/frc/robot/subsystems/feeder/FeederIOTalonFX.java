package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class FeederIOTalonFX implements FeederIO {
    TalonFX m_feeder, m_column;

    private final StatusSignal<Voltage> feederVoltage, columnVoltage;
    private final StatusSignal<Current> feederSupplyCurrent, feederStatorCurrent,
            columnSupplyCurrent, columnStatorCurrent;

    public FeederIOTalonFX() {
        m_feeder = new TalonFX(FeederConstants.FEEDER_ID);
        m_column = new TalonFX(FeederConstants.COLUMN_ID);

        m_feeder.getConfigurator().apply(FeederConstants.CONFIG);
        m_column.getConfigurator().apply(FeederConstants.CONFIG.withMotorOutput(new MotorOutputConfigs()));

        feederVoltage = m_feeder.getMotorVoltage();
        feederSupplyCurrent = m_feeder.getSupplyCurrent();
        feederStatorCurrent = m_feeder.getStatorCurrent();

        columnVoltage = m_column.getMotorVoltage();
        columnSupplyCurrent = m_column.getSupplyCurrent();
        columnStatorCurrent = m_column.getStatorCurrent();

        // set refresh rates and clear all other signals from the CAN bus
        BaseStatusSignal.setUpdateFrequencyForAll(20,
                feederVoltage,
                feederSupplyCurrent,
                feederStatorCurrent,
                columnVoltage,
                columnSupplyCurrent,
                columnStatorCurrent
        );

        m_feeder.optimizeBusUtilization();
        m_column.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                feederVoltage,
                feederSupplyCurrent,
                feederStatorCurrent,
                columnVoltage,
                columnSupplyCurrent,
                columnStatorCurrent
        );

        inputs.feederVoltage = feederVoltage.getValue();
        inputs.feederSupplyCurrent = feederSupplyCurrent.getValue();
        inputs.feederStatorCurrent = feederStatorCurrent.getValue();

        inputs.columnVoltage = columnVoltage.getValue();
        inputs.columnSupplyCurrent = columnSupplyCurrent.getValue();
        inputs.columnStatorCurrent = columnStatorCurrent.getValue();
    }

    @Override
    public void setFeederVoltage(Voltage voltage) {
        m_feeder.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setColumnVoltage(Voltage voltage) {
        m_column.setVoltage(voltage.in(Volts));
    }
}
