package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

import java.util.Queue;

import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * IO implementation for TalonFX-based swerve module.
 * Manages drive and steer motors with CANcoder absolute encoder.
 */
public class ModuleIOTalonFX implements ModuleIO {
  // Hardware
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final DriveConstants.ModuleConstants config;

  // Control requests
  private final VelocityVoltage driveRequest;
  private final VelocityTorqueCurrentFOC driveTorqueRequest;
  private final PositionVoltage steerRequest;

  // Drive motor signals
  private final StatusSignal<Angle> drivePositionSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Voltage> driveVoltageSignal;
  private final StatusSignal<Current> driveCurrentSignal;

  // Steer motor signals
  private final StatusSignal<Angle> steerPositionSignal;
  private final StatusSignal<AngularVelocity> steerVelocitySignal;
  private final StatusSignal<Voltage> steerVoltageSignal;
  private final StatusSignal<Current> steerCurrentSignal;

  // Encoder signal
  private final StatusSignal<Angle> steerAbsolutePositionSignal;

  // Odometry queues for high-frequency sampling
  private final Queue<Double> odometryTimestampQueue;
  private final Queue<Double> odometrySteerPositionQueue;
  private final Queue<Double> odometryDrivePositionQueue;

  public ModuleIOTalonFX(DriveConstants.ModuleConstants config) {
    this.config = config;

    // Initialize hardware
    driveMotor = new TalonFX(config.driveId(), new CANBus("canivore"));
    steerMotor = new TalonFX(config.steerId(), new CANBus("canivore"));
    encoder = new CANcoder(config.encoderId(), new CANBus("canivore"));

    configureDevices();

    // Sync steer motor position with absolute encoder
    Timer.delay(0.5);
    steerMotor.setPosition(encoder.getAbsolutePosition().getValueAsDouble()
      - config.encoderOffset().getRotations());

    // Initialize control requests
    driveRequest = new VelocityVoltage(0.0)
      .withEnableFOC(true)
      .withSlot(0)
      .withUpdateFreqHz(0);
    driveTorqueRequest = new VelocityTorqueCurrentFOC(0.0)
      .withSlot(1);
    steerRequest = new PositionVoltage(0.0)
      .withEnableFOC(true)
      .withSlot(0)
      .withUpdateFreqHz(0);

    // Initialize status signals
    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();
    driveVoltageSignal = driveMotor.getMotorVoltage();
    driveCurrentSignal = driveMotor.getStatorCurrent();

    steerPositionSignal = steerMotor.getPosition();
    steerVelocitySignal = steerMotor.getVelocity();
    steerVoltageSignal = steerMotor.getMotorVoltage();
    steerCurrentSignal = steerMotor.getStatorCurrent();

    steerAbsolutePositionSignal = encoder.getAbsolutePosition();

    // Configure update frequencies (50 Hz for standard signals)
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      driveVelocitySignal,
      driveCurrentSignal,
      steerVelocitySignal,
      steerVoltageSignal,
      steerCurrentSignal,
      steerAbsolutePositionSignal
    );

    // High-frequency odometry signals
    BaseStatusSignal.setUpdateFrequencyForAll(
      DriveConstants.ODOMETRY_FREQUENCY, drivePositionSignal, steerPositionSignal
    );

    // Register odometry queues
    odometryTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    odometrySteerPositionQueue = PhoenixOdometryThread.getInstance()
      .registerSignal(steerPositionSignal);
    odometryDrivePositionQueue = PhoenixOdometryThread.getInstance()
      .registerSignal(drivePositionSignal);

    // Optimize CAN bus utilization
    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    // Refresh all status signals
    BaseStatusSignal.refreshAll(
      drivePositionSignal,
      driveVelocitySignal,
      driveVelocitySignal,
      driveCurrentSignal,
      steerPositionSignal,
      steerVelocitySignal,
      steerVoltageSignal,
      steerCurrentSignal,
      steerAbsolutePositionSignal
    );

    // Update drive motor inputs
    inputs.driveConnected = driveMotor.isConnected();
    inputs.drivePositionRads = Units.rotationsToRadians(drivePositionSignal.getValueAsDouble());
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocitySignal.getValueAsDouble());
    inputs.driveAppliedVolts = driveVoltageSignal.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentSignal.getValueAsDouble();

    // Update steer motor inputs
    inputs.steerConnected = steerMotor.isConnected();
    inputs.steerPosition = Rotation2d.fromRotations(steerPositionSignal.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocitySignal.getValueAsDouble());
    inputs.steerAppliedVolts = steerVoltageSignal.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrentSignal.getValueAsDouble();
    inputs.steerAbsolutePosition = Rotation2d.fromRotations(
      steerAbsolutePositionSignal.getValueAsDouble() - config.encoderOffset().getRotations());

    // Process odometry queues
    inputs.odometryTimestamps =
      odometryTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRads =
      odometryDrivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometrySteerPositions =
      odometrySteerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    // Clear queues for next cycle
    odometryTimestampQueue.clear();
    odometrySteerPositionQueue.clear();
    odometryDrivePositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double radsPerSec, Current feedforward) {
//    driveMotor.setControl(driveRequest.withVelocity(RadiansPerSecond.of(radsPerSec)));
    driveMotor.setControl(
        driveTorqueRequest
            .withVelocity(RadiansPerSecond.of(radsPerSec))
            .withFeedForward(feedforward)
    );
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerMotor.setControl(steerRequest.withPosition(rotation.getRotations()));
  }

  /**
   * Configures all hardware devices with motor controllers and encoder settings.
   */
  private void configureDevices() {
    // Drive motor configuration
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    // Drive current limits
    motorConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(70)
      .withSupplyCurrentLowerTime(1.0)
      .withSupplyCurrentLowerLimit(40)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(100);

    // Drive PID and feedforward (Slot 0: voltage control)
    motorConfig.Slot0.withKP(0.25)
      .withKD(0.0)
      .withKS(0.15173)
      .withKV(0.69406)
      .withKA(0.0);

    // Slot 1: torque current FOC control
    motorConfig.Slot1.withKP(62.5)
      .withKD(0.0)
      .withKS(0.14957)
      .withKV(0.71149)
      .withKA(0.0);

    driveMotor.getConfigurator().apply(motorConfig);

    // Steer motor configuration
    motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.STEER_GEAR_RATIO;
    motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.CurrentLimits.withStatorCurrentLimit(60);
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    motorConfig.Slot0.withKP(80.0)
      .withKD(1.0)
      .withKS(0.0)
      .withKV(0.0);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    steerMotor.getConfigurator().apply(motorConfig);

    // Reset motor positions
    driveMotor.setPosition(0.0);
    steerMotor.setPosition(0.0);

    // Configure encoder
    var encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
  }
}
