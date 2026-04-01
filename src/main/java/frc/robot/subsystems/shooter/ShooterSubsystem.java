package frc.robot.subsystems.shooter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
    // IO layer handles all talking to real hardware
    private final ShooterIO m_io;

    // IO inputs class handles getting data from hardware
    private final ShooterIOInputsAutoLogged m_inputs;

    private final LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("Shooter RPM", 3000);

    public ShooterSubsystem(ShooterIO io) {
        m_io = io;
        m_inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // update all our input values from our hardware/sim
        m_io.updateInputs(m_inputs);
        Logger.processInputs("shooter", m_inputs);
    }

    public boolean flywheelAtRPM() {
        return MathUtil.isNear(RobotState.getInstance().getShooterRPM(), m_inputs.shooterSpeedL.in(RPM), 100)
            && MathUtil.isNear(RobotState.getInstance().getShooterRPM(), m_inputs.shooterSpeedM.in(RPM), 100)
            && MathUtil.isNear(RobotState.getInstance().getShooterRPM(), m_inputs.shooterSpeedR.in(RPM), 100);
    }

    // Commands are what get bound to buttons
    // They use lambas, essentially using a function as data
    public Command setShooterVoltage(Voltage voltage) {
        return runEnd(
                // Start conditions
                () -> m_io.setShooterVoltage(voltage),
                // End conditions
                () -> m_io.setShooterVoltage(Volt.of(0.0))
        );
    }

    public Command runShooterRPM(AngularVelocity rpm) {
        return runEnd(
            () -> m_io.setShooterRPM(rpm),
            () -> m_io.setShooterVoltage(Volts.of(0.0))
        ).withName("Run Shooter RPM");
    }

    public Command runDashboardRPM() {
        return runEnd(
                () -> m_io.setShooterRPM(RPM.of(shooterSpeed.getAsDouble())),
                () -> m_io.setShooterVoltage(Volts.of(0.0))
        ).withName("Run Dashboard RPM");
    }

    public Command setHoodPosition(double position) {
        return runOnce(() -> m_io.setHoodDistance(position)).withName("Hood Position");
    }

    public Command spinupFlywheel() {
        return run(
            () -> {
                m_io.setShooterRPM(RPM.of(RobotState.getInstance().getShooterRPM()));
                m_io.setHoodDistance(RobotState.getInstance().getHoodAngle());
            }
        );
    }

    public Command autoAim() {
        return runEnd(
            () -> {
                m_io.setShooterRPM(RPM.of(RobotState.getInstance().getShooterRPM()));
                m_io.setHoodDistance(RobotState.getInstance().getHoodAngle());
            },
            () -> {
                m_io.setShooterVoltage(Volts.of(0.0));
                m_io.setHoodDistance(0.1);
            }).withName("Shooter Auto Aim");
    }

    public Command autoAimOnMove() {
        return runEnd(
            () -> {
                m_io.setShooterRPM(RPM.of(RobotState.getInstance().getShootOnMoveShotData().shot().shooterRPM));
                m_io.setHoodDistance(RobotState.getInstance().getShootOnMoveShotData().shot().hoodPosition);
            },
            () -> {
                m_io.setShooterVoltage(Volts.of(0.0));
                m_io.setHoodDistance(0.1);
            }).withName("Shooter Auto Aim");
    }

    public Command shooterAutoHood() {
        return run(() -> {
            if ((RobotState.getInstance().getEstimatedPose().getX() > FieldConstants.LinesVertical.neutralZoneNear) &&
                (RobotState.getInstance().getEstimatedPose().getX() < FieldConstants.LinesVertical.neutralZoneFar)) {
                m_io.setHoodDistance(0.8);
            } else {
                m_io.setHoodDistance(0.1);
            }
        });
    }
}

