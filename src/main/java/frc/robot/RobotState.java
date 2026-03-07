package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;
//import frc.robot.util.AllianceFlipUtil;
//import frc.robot.util.FieldConstants;
//import frc.robot.util.FieldRelativeSpeeds;
import lombok.Getter;
import lombok.Setter;
//import org.dyn4j.geometry.Polygon;
//import org.dyn4j.geometry.Vector2;
//import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
//import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

public class RobotState {

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose estimation
//  @Getter
//  @AutoLogOutput(key = "RobotState/OdometryPose")
//  private Pose2d odometryPose = new Pose2d();

  private Rotation2d lastRawGyro = new Rotation2d();

  // use for simulation
//  @Setter
//  private Optional<SwerveDriveSimulation> driveSimulation = Optional.empty();

  private final LoggedNetworkBoolean useAuto =
      new LoggedNetworkBoolean("Smartdashboard/Use Auto?", true);

  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS),
          new Rotation2d(),
          new SwerveModulePosition[]{
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.01, 0.01, 0.02),
          VecBuilder.fill(0.05, 0.05, 0.03)
      );

  // used to filter vision measurements into odometry estimation
  // Odometry
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  // Shooter lookup tables
  private InterpolatingDoubleTreeMap shooterSpeedDistanceMap =
          new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterHoodDistanceMap =
          new InterpolatingDoubleTreeMap();

  private RobotState() {
    AutoLogOutputManager.addObject(this);

    // setup lookup tables
    shooterSpeedDistanceMap.put(0.0, 0.0);
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(lastRawGyro, lastWheelPositions, pose);

//    driveSimulation.ifPresent(swerveDriveSimulation -> swerveDriveSimulation.setSimulationWorldPose(pose));
  }

  public void addOdometryMeasurement(Rotation2d heading, SwerveModulePosition[] modulePositions, double timestamp) {
    lastRawGyro = heading;
    lastWheelPositions = modulePositions;
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
  }

  public void addVisionMeasurement(VisionObservation update) {
    poseEstimator.addVisionMeasurement(update.visionPose, update.timestamp, update.stdDevs);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
//    if (driveSimulation.isPresent()) {
//      return driveSimulation.get().getSimulatedDriveTrainPose();
//    }
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /** Gets the distance to the hub shooter */
  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToHubInches() {
    return FieldConstants.Hub.innerCenterPoint.toTranslation2d()
        .getDistance(getEstimatedPose().getTranslation());
  }

  /** Gets the angle from the robot to a certain point on the field. Used for auto aim */
  public Rotation2d getPointAtAngle(Translation2d point) {
    Translation2d robotToPoint = point.minus(getEstimatedPose().getTranslation());
    Rotation2d angleToPoint = new Rotation2d(robotToPoint.getX(), robotToPoint.getY());
    Logger.recordOutput("RobotState/AngleToPoint", angleToPoint);
    return angleToPoint;
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
