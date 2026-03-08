package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.DriveConstants;
//import frc.robot.util.AllianceFlipUtil;
//import frc.robot.util.FieldConstants;
//import frc.robot.util.FieldRelativeSpeeds;
//import org.dyn4j.geometry.Polygon;
//import org.dyn4j.geometry.Vector2;
//import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
//import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.units.Units.*;

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

  // Shoot on the move
  private Pose2d lastPose = new Pose2d();
  private Translation2d fieldRelativeVelocity = new Translation2d();

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

  private final LoggedNetworkNumber lookaheadTime =
      new LoggedNetworkNumber("ShootOnMove/LookaheadTime", 0.02);

  // Shooter lookup tables
  private InterpolatingDoubleTreeMap shooterSpeedDistanceMap =
          new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterHoodDistanceMap =
          new InterpolatingDoubleTreeMap();

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }
  }

  private final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
      (startValue, endValue, q) ->
          InverseInterpolator.forDouble()
              .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
      (startValue, endValue, t) ->
          new Shot(
              Interpolator.forDouble()
                  .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
              Interpolator.forDouble()
                  .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
          )
  );

  private RobotState() {
    AutoLogOutputManager.addObject(this);

    // setup lookup tables
    shooterSpeedDistanceMap.put(1.56, 3000.0);
    shooterSpeedDistanceMap.put(2.19, 3000.0);
    shooterSpeedDistanceMap.put(2.94, 3000.0);
    shooterSpeedDistanceMap.put(3.19, 3250.0);
    shooterSpeedDistanceMap.put(1.56, 2750.0);
    shooterSpeedDistanceMap.put(1.32, 2750.0);
    shooterSpeedDistanceMap.put(1.63, 2750.0);
    shooterSpeedDistanceMap.put(1.63, 3000.0);
    shooterSpeedDistanceMap.put(2.14, 3000.0);
    shooterSpeedDistanceMap.put(2.41, 3000.0);
    shooterSpeedDistanceMap.put(2.63, 3250.0);
    shooterSpeedDistanceMap.put(2.63, 3125.0);
    shooterSpeedDistanceMap.put(3.26, 3500.0);
    shooterSpeedDistanceMap.put(3.26, 3250.0);

    shooterHoodDistanceMap.put(2.19, 0.5);
    shooterHoodDistanceMap.put(2.94, 0.5);
    shooterHoodDistanceMap.put(3.19, 0.5);
    shooterHoodDistanceMap.put(1.56, 0.1);
    shooterHoodDistanceMap.put(1.56, 0.1);
    shooterHoodDistanceMap.put(1.32, 0.1);
    shooterHoodDistanceMap.put(1.63, 0.3);
    shooterHoodDistanceMap.put(1.63, 0.1);
    shooterHoodDistanceMap.put(2.14, 0.3);
    shooterHoodDistanceMap.put(2.41, 0.3);
    shooterHoodDistanceMap.put(2.63, 0.3);
    shooterHoodDistanceMap.put(2.63, 0.3);
    shooterHoodDistanceMap.put(3.26, 0.5);
    shooterHoodDistanceMap.put(3.26, 0.5);

    // WCP values
    distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
    distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
    distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));

    // Our added values
    distanceToShotMap.put(Meters.of(2.60), new Shot(3200, 0.36));
  }

  public void updateVelocityPeriodic() {
    fieldRelativeVelocity = getEstimatedPose()
        .getTranslation()
        .minus(lastPose.getTranslation())
        .div(0.02); // loop cycle
    lastPose = getEstimatedPose();
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

  /** Gets the needed hood angle for a certain distance */
  @AutoLogOutput(key = "RobotState/HoodPercent")
  public double getHoodAngle() {
//    return shooterHoodDistanceMap.get(getDistanceToHubInches());
    return distanceToShotMap.get(Meters.of(getDistanceToHubMeters())).hoodPosition;
  }

  /** Gets the needed shooter RPM angle for a certain distance */
  @AutoLogOutput(key = "RobotState/ShooterRPM")
  public double getShooterRPM() {
//    return shooterSpeedDistanceMap.get(getDistanceToHubMeters());
    return distanceToShotMap.get(Meters.of(getDistanceToHubMeters())).shooterRPM;
  }

  /** Gets the distance to the hub shooter */
  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToHubMeters() {
    return FieldConstants.Hub.goalPoint
        .getDistance(getEstimatedPose().getTranslation());
  }

  /** Gets the angle from the robot to a certain point on the field. Used for auto aim */
  public Rotation2d getPointAtAngle(Translation2d point) {
    Translation2d robotToPoint = point.minus(getEstimatedPose().getTranslation());
    Rotation2d angleToPoint = new Rotation2d(robotToPoint.getX(), robotToPoint.getY());
    Logger.recordOutput("RobotState/AngleToPoint", angleToPoint);
    return angleToPoint;
  }

  private double hoodToRadian(double percent) {
    return Units.degreesToRadians(38.0 * percent);
  }

  public ShotData getShootOnMoveShotData() {
    // uses the derivation from https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472/11
    // static shot parameters

    /*
    double v = Units.rotationsPerMinuteToRadiansPerSecond(getShooterRPM());
    double phi_v = getHoodAngle();
    double phi_h = getPointAtAngle(FieldConstants.Hub.goalPoint).getRadians();

    // robot velocity
    double v_x = fieldRelativeVelocity.getX();
    double v_y = fieldRelativeVelocity.getY();

    // calculate offsets to cancel out robot velocity
    // robot angle
    double theta_h = Math.atan2(
        (v * Math.cos(phi_v) * Math.sin(phi_h)) + v_y,
        (v * Math.cos(phi_v) * Math.sin(phi_h)) + v_x
    );

    // hood angle
    double theta_v = Math.atan2(
        v * Math.sin(phi_v) * Math.cos(theta_h),
        (v * Math.cos(phi_v) * Math.cos(phi_h)) + v_x
    );

    // shot rad/sec
    double v_s = v * (Math.sin(phi_v) / Math.sin(theta_v));

    Logger.recordOutput("ShootOnMove/ShotAngle", Rotation2d.fromRadians(theta_h));
    Logger.recordOutput("ShootOnMove/ShooterRPM", Units.radiansPerSecondToRotationsPerMinute(v_s));
    Logger.recordOutput("ShootOnMove/HoodPercent", Units.radiansToDegrees(theta_h) / 38.0);


    Logger.recordOutput("ShootOnMove/Robot Velocity", fieldRelativeVelocity);
    */

    // look ahead to the future goal
    double lookahead = lookaheadTime.getAsDouble();
    Translation2d lookaheadPoint = FieldConstants.Hub.goalPoint
        .minus(fieldRelativeVelocity.times(lookahead));
    double effectiveDistance = lookaheadPoint.getDistance(getEstimatedPose().getTranslation());

    // Find the angle to the future goal
    Translation2d robotToPoint = lookaheadPoint.minus(getEstimatedPose().getTranslation());
    Rotation2d angleToPoint = new Rotation2d(robotToPoint.getX(), robotToPoint.getY());

    // Get the new shot data
    Shot shot = distanceToShotMap.get(Meters.of(effectiveDistance));

    Logger.recordOutput("ShootOnMove/Goal", new Translation3d(lookaheadPoint));
    Logger.recordOutput("ShootOnMove/Angle", angleToPoint);
    Logger.recordOutput("ShootOnMove/Hood", shot.hoodPosition);
    Logger.recordOutput("ShootOnMove/RPM", shot.shooterRPM);
    Logger.recordOutput("ShootOnMove/EffectiveDistance", Meters.of(effectiveDistance));
    Logger.recordOutput("ShootOnMove/DeltaGoal", lookaheadPoint.minus(FieldConstants.Hub.goalPoint));

    return new ShotData(
        angleToPoint,
        shot
    );
  }

  public record ShotData(Rotation2d shotAngle, Shot shot) {}
  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
