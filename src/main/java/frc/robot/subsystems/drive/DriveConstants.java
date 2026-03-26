package frc.robot.subsystems.drive;

//import com.pathplanner.lib.config.ModuleConfig;
//import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;

import static edu.wpi.first.units.Units.*;

/**
 * Constants for the swerve drive subsystem.
 * Includes module positions, gear ratios, and control parameters.
 */
public class DriveConstants {
  // Odometry update frequency (Hz)
  public static final double ODOMETRY_FREQUENCY = 250;

  // Robot dimensions
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(19.25);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(17.5);

  // Module positions (FL, FR, BL, BR)
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
    new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
    new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
    new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
  };

  // Drive base geometry
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(17.1);
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS);

  // Wheel and gear ratios
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double STEER_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0); // 18.75

  // Traction coefficient
  public static final double WHEEL_COF = 1.2;

  // Translation PID constants
  public static final double XY_KP = 3.75;
  public static final double XY_KI = 0.0;
  public static final double XY_KD = 0.01;

  // Rotation PID constants
  public static final double THETA_KP = 3.5;
  public static final double THETA_KI = 0.0;
  public static final double THETA_KD = 0.1;

  // Trajectory constraints
  public static final TrajectoryConfig TRAJECTORY_CONFIG =
    new TrajectoryConfig(Units.feetToMeters(4.25), Units.feetToMeters(4.25));
  public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
    new TrapezoidProfile.Constraints(4.5 * Math.PI, 4.5 * Math.PI); // rad/s, rad/s^2

//  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
//      .withCustomModuleTranslations(MODULE_TRANSLATIONS)
//      .withRobotMass(Pounds.of(115))
//      .withGyro(COTS.ofPigeon2())
//      .withSwerveModule(new SwerveModuleSimulationConfig(
//          DCMotor.getKrakenX60(1),
//          DCMotor.getKrakenX60(1),
//          DRIVE_GEAR_RATIO,
//          STEER_GEAR_RATIO,
//          Volts.of(0.1),
//          Volts.of(0.1),
//          Meters.of(WHEEL_RADIUS_METERS),
//          KilogramSquareMeters.of(0.02),
//          WHEEL_COF));

  public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
      Pounds.of(115),
      KilogramSquareMeters.of(6),
      new ModuleConfig(
          WHEEL_RADIUS_METERS,
          MAX_LINEAR_SPEED_MPS / 4,
          1.2,
          DCMotor.getKrakenX60Foc(1)
              .withReduction(DRIVE_GEAR_RATIO),
          70,
          1
      ),
      MODULE_TRANSLATIONS
  );

  public static final ModuleConstants[] MODULE_CONSTANTS;


  // Initialize module constants based on robot mode
  static {
    switch (Constants.getMode()) {
      case REAL -> {
        MODULE_CONSTANTS = new ModuleConstants[]{
          // Front Left
          ModuleConstants.builder()
            .driveId(1)
            .steerId(2)
            .encoderId(3)
            .encoderOffset(Rotation2d.fromRotations(-0.057861).plus(Rotation2d.k180deg))
            .steerInverted(true)
            .turnInverted(false)
            .build(),
          // Front Right
          ModuleConstants.builder()
            .driveId(4)
            .steerId(5)
            .encoderId(6)
            .encoderOffset(Rotation2d.fromRotations(0.029785).plus(Rotation2d.k180deg))
            .steerInverted(true)
            .turnInverted(false)
            .build(),
          // Back Left
          ModuleConstants.builder()
            .driveId(7)
            .steerId(8)
            .encoderId(9)
            .encoderOffset(Rotation2d.fromRotations(0.49560).plus(Rotation2d.k180deg))
            .steerInverted(true)
            .turnInverted(false)
            .build(),
          // Back Right
          ModuleConstants.builder()
            .driveId(10)
            .steerId(11)
            .encoderId(12)
            .encoderOffset(Rotation2d.fromRotations(0.190918).plus(Rotation2d.k180deg))
            .steerInverted(true)
            .turnInverted(false)
            .build()
        };
      }
      default -> {
        MODULE_CONSTANTS = new ModuleConstants[]{};
      }
    }
  }

  /**
   * Configuration for a single swerve module.
   * Contains CAN IDs, encoder offset, and inversion settings.
   */
  @Builder
  public record ModuleConstants(
    int driveId,
    int steerId,
    int encoderId,
    Rotation2d encoderOffset,
    boolean turnInverted,
    boolean steerInverted
  ) {}
}
