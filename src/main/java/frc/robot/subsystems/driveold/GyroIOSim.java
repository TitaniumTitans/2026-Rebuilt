package frc.robot.subsystems.driveold;

public class GyroIOSim implements GyroIO {
//  private final GyroSimulation gyroSimulation;

//  public GyroIOSim(GyroSimulation gyroSimulation) {
//    this.gyroSimulation = gyroSimulation;
//  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    inputs.connected = true;
//    inputs.yawPosition = gyroSimulation.getGyroReading();
//    inputs.yawVelocityRadsPerSec =
//        gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
//
//    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
//    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
  }
}