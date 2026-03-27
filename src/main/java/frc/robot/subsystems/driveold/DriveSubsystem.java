package frc.robot.subsystems.driveold;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.driveold.module.Module;
import frc.robot.subsystems.driveold.module.ModuleIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.driveold.DriveConstants.ROBOT_CONFIG;
//import static frc.robot.subsystems.drive.DriveConstants.ROBOT_CONFIG;

/**
 * Swerve drive subsystem.
 * Manages four swerve modules, odometry, and driving commands.
 */
public class DriveSubsystem extends SubsystemBase {
  // Odometry lock for thread safety
  public static final Lock odometryLock = new ReentrantLock();

  // Hardware IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  // Alerts
  private final Alert gyroDisconnectAlert =
    new Alert("Gyro disconnected, falling back to kinematics.", Alert.AlertType.kError);

  // Kinematics and odometry
  private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);
  private final SwerveDriveOdometry wpiOdom;

  // System identification
  private final SysIdRoutine sysId;

  /**
   * Constructs the drive subsystem.
   * Initializes modules, odometry, and SysId routine.
   */
  public DriveSubsystem(GyroIO gyro,
                        ModuleIO flModuleIo,
                        ModuleIO frModuleIo,
                        ModuleIO blModuleIo,
                        ModuleIO brModuleIo) {
    AutoLogOutputManager.addObject(this);

    this.gyroIO = gyro;
    modules[0] = new Module(flModuleIo, 0);
    modules[1] = new Module(frModuleIo, 1);
    modules[2] = new Module(blModuleIo, 2);
    modules[3] = new Module(brModuleIo, 3);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    wpiOdom = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());

//    RobotState.getInstance().resetPose(new Pose2d());

    AutoBuilder.configure(
        RobotState.getInstance()::getEstimatedPose,
        this::resetPose,
        this::getChassisSpeeds,
        (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> this.runVelocity(speeds, feedforwards),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0),
            new PIDConstants(5.0, 0.0)
        ),
        ROBOT_CONFIG,
        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
        this
    );

    PathPlannerLogging.setLogActivePathCallback(
        (List<Pose2d> path) -> Logger.recordOutput("PathPlanner/ActivePath", path.toArray(Pose2d[]::new))
    );
    PathPlannerLogging.setLogCurrentPoseCallback(
        (Pose2d pose) -> Logger.recordOutput("PathPlanner/CurrentPose", pose)
    );
    PathPlannerLogging.setLogTargetPoseCallback(
        (Pose2d pose) -> Logger.recordOutput("PathPlanner/TargetPose", pose)
    );

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    // Run module periodic
    for (var module : modules) {
      module.periodic();
    }

    // Stop modules when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // Log empty setpoints when disabled
      Logger.recordOutput("SwerveStates/Optimized", new SwerveModuleState[] {});
    }

    // Process odometry measurements
    double[] timestamps = modules[0].getOdometryTimestamps();
    int timestampLength = Math.min(timestamps.length, gyroInputs.odometryYawTimestamps.length);
    for (int i = 0; i < timestampLength; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
        .addOdometryMeasurement(gyroInputs.odometryYawPositions[i],
          wheelPositions,
          timestamps[i]);
    }

//    RobotState.getInstance().addNormalMeasurement(gyroInputs.yawPosition, getModulePositions());
    // Update WPILib odometry
    wpiOdom.update(getGyroRotation(), getModulePositions());
    Logger.recordOutput("RobotState/WPIOdometry", wpiOdom.getPoseMeters());

    // Update gyro disconnection alert
    gyroDisconnectAlert.set(!gyroInputs.connected && Constants.getMode() != Constants.Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, DriveFeedforwards.zeros(4));
  }

  // runs the drivetrain at a set chassis speed
  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // calculate module setpoints
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED_MPS);

    // Log setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveSpeeds/Setpoints", speeds);
    Logger.recordOutput("SwerveSpeeds/Optimized", discretizedSpeeds);
    Logger.recordOutput("SwerveStates/UsingSetpoints", false);
    Logger.recordOutput("SwerveStates/Actual Setpoints", setpointStates);
    Logger.recordOutput("PathPlanner/FeedforwardsMetersPerSecondSquared", feedforwards.accelerationsMPSSq());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], feedforwards.torqueCurrents()[i]);
    }

    // Log optimized setpoints (runSetpoint mutates the state)
    Logger.recordOutput("SwerveStates/Optimized", setpointStates);
  }

  /** Resets the gyro to the specified angle. */
  public void resetGyro(Rotation2d angle) {
    gyroIO.reset(angle);
  }

  /** Runs characterization at the specified voltage. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops all module motion. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the current measured state of all modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the absolute measured state of all modules. */
  @AutoLogOutput(key = "SwerveStates/AbsoluteMeasured")
  private SwerveModuleState[] getAbsoluteModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getAbsoluteState();
    }
    return states;
  }

  /** Returns the current position of all modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current robot-relative chassis speeds. */
  @AutoLogOutput(key = "SwerveSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current field-relative chassis speeds. */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
      kinematics.toChassisSpeeds(getModuleStates()),
      RobotState.getInstance().getRotation());
  }

  /** Returns wheel positions for radius characterization. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns average velocity for feedforward characterization. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current gyro rotation. */
  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

  /** Returns the maximum linear speed in m/s. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED_MPS;
  }

  /** Returns the slow linear speed (25% of max). */
  public double getSlowLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED_MPS * 0.25;
  }

  /** Returns the maximum angular speed in rad/s. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns the slow angular speed (25% of max). */
  public double getSlowAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED * 0.25;
  }

  public Command driveToPose(Pose2d pose) {
    return driveToPose(() -> pose);
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return AutoBuilder.pathfindToPose(
        pose.get(),
        new PathConstraints(1.25, 1.25,
            1.25, 1.25)
    ).withName("Pathfinding Command")
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /** Resets the robot pose to the specified position. */
  public void resetPose(Pose2d pose) {
    Logger.recordOutput("Pose Reset To", pose);
    RobotState.getInstance().resetPose(pose);
  }

  /** Returns a command to reset the pose to the specified position. */
  public Command resetPoseFactory(Pose2d pose) {
    return resetPoseFactory(() -> pose);
  }

  /** Returns a command to reset the pose using a supplier. */
  public Command resetPoseFactory(Supplier<Pose2d> pose) {
    return runOnce(() -> resetPose(pose.get()));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(sysId.dynamic(direction));
  }
}
