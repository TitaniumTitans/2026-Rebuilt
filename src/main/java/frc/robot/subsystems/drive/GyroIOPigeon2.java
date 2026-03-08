package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Queue;

/**
 * IO implementation for Pigeon 2 gyroscope.
 * Provides yaw angle and velocity measurements for odometry.
 */
public class GyroIOPigeon2 implements GyroIO {
  // Hardware
  private final Pigeon2 pigeon = new Pigeon2(13, new CANBus("canivore"));

  // Status signals
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  // Odometry queues for high-frequency measurements
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2() {
    // Apply default configuration and zero the gyro
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    // Configure update frequencies (250 Hz for odometry, 50 Hz for velocity)
    yaw.setUpdateFrequency(250);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();

    // Register signals with odometry thread
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    // Refresh all signals and check connection status
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadsPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // Process odometry measurements from queues
    inputs.odometryYawPositions = yawPositionQueue.stream()
      .map(Rotation2d::fromDegrees)
      .toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps = yawTimestampQueue.stream()
      .mapToDouble((Double value) -> value)
      .toArray();

    // Clear queues for next cycle
    yawPositionQueue.clear();
    yawTimestampQueue.clear();
  }

  @Override
  public void reset(Rotation2d angle) {
    pigeon.setYaw(angle.getMeasure());
  }
}