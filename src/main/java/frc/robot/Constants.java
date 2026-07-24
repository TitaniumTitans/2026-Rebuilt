// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.BiAlliancePose3d;
import frc.robot.util.InterpolatingMeasureTreeMap;
import frc.robot.util.PoseUtil;
import frc.robot.util.RectangleUtil;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.*;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  @AutoLogOutput(key = "RobotMode")
  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else {
      return Mode.SIM;
    }
  }

  public static final CANBus CANIVORE_BUS = new CANBus("canivore");
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  // yoinked constants from apollo
  /**
   * Constants that describe the physical layout of the field.
   */
  public static class FieldConstants {
    /**
     * AprilTag Field Layout for the current game.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    /**
     * A pose at the center of the field.
     */
    public static final Pose2d FIELD_CENTER = new Pose2d(FIELD_LAYOUT.getFieldLength() / 2, FIELD_LAYOUT.getFieldWidth() / 2, Rotation2d.kZero);
    /**
     * A rectangle encompassing the neutral (center) zone of the field.
     */
    public static final Rectangle2d NEUTRAL_RECTANGLE = new Rectangle2d(FIELD_CENTER, Inches.of(287.0), Meters.of(FIELD_LAYOUT.getFieldWidth()));

    /**
     * Zones and poses to shoot from and to.
     */
    public static class ShootingZones {
      /**
       * The pose to shoot at for the top of the neutral rectangle (when looking at the field diagram).
       */
      public static final BiAlliancePose3d SHUTTLE_TOP_POSE = BiAlliancePose3d.fromBluePose(new Pose3d(Meters.of(3.0), Meters.of(2.0), Meters.zero(), Rotation3d.kZero), BiAlliancePose3d.InvertY.KEEP_Y);
      /**
       * The pose to shoot at for the bottom of the neutral rectangle (when looking at the field diagram).
       */
      public static final BiAlliancePose3d SHUTTLE_BOTTOM_POSE = BiAlliancePose3d.fromBluePose(PoseUtil.flipPoseY(SHUTTLE_TOP_POSE.getBluePose()), BiAlliancePose3d.InvertY.KEEP_Y);
      /**
       * The pose to shoot at into the top of the neutral rectangle (when looking at the field diagram). This is used when shooting from the other alliance's hub.
       */
      public static final Pose3d SHUTTLE_CENTER_TOP_POSE = new Pose3d(FIELD_CENTER.getMeasureX(), Meters.of(2.0), Meters.zero(), Rotation3d.kZero);
      /**
       * The pose to shoot at into the bottom of the neutral rectangle (when looking at the field diagram). This is used when shooting from the other alliance's hub.
       */
      public static final Pose3d SHUTTLE_CENTER_BOTTOM_POSE = PoseUtil.flipPoseY(SHUTTLE_CENTER_TOP_POSE);

      /**
       * A rectangle encompassing the shooting zone for the hub on the red alliance.
       */
      public static final Rectangle2d HUB_ZONE_RED = new Rectangle2d(FIELD_CENTER.transformBy(new Transform2d(Inches.of(234.555).plus(Meters.of(0.1)), Meters.zero(), Rotation2d.kZero)), Inches.of(182.11), Inches.of(317.7));
      /**
       * A rectangle encompassing the shooting zone for the hub on the blue alliance.
       */
      public static final Rectangle2d HUB_ZONE_BLUE = RectangleUtil.flipRectangleX(HUB_ZONE_RED);
      /**
       * The pose to shoot at for the {@link #HUB_ZONE_RED} and {@link #HUB_ZONE_BLUE}.
       */
      public static final BiAlliancePose3d HUB_POSE = BiAlliancePose3d.fromRedPose(new Pose3d(FIELD_CENTER).transformBy(new Transform3d(Inches.of(143.50), Meters.zero(), Inches.of(72.0), Rotation3d.kZero)), BiAlliancePose3d.InvertY.KEEP_Y);
    }
  }

  /**
   * Constants that control the shooting behavior.
   */
  public static class ShootingConstants {
    // TODO: Update all of these constants
    /**
     * An interpolation table used for flywheel speed by gamepiece velocity.
     */
    public static final InterpolatingMeasureTreeMap<LinearVelocity, LinearVelocityUnit, AngularVelocity, AngularVelocityUnit> FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY = new InterpolatingMeasureTreeMap<>();

    static {
      // Add values to the interpolation table
      FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY.put(MetersPerSecond.of(0.0), RPM.of(10.0));
      FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY.put(MetersPerSecond.of(10.0), RPM.of(50.0));
    }

    /**
     * An interpolation table used for hood angle by distance, for basic shoot-from-anywhere.
     */
    public static final InterpolatingMeasureTreeMap<Distance, DistanceUnit, Angle, AngleUnit> HOOD_ANGLE_BY_DISTANCE = new InterpolatingMeasureTreeMap<>();

    static {
      // Add values to the interpolation table
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(1.9), Degrees.of(0));
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(2.58), Degrees.of(0));
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(3.29), Degrees.of(2));
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(4.5), Degrees.of(4));
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(5.29), Degrees.of(5));
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(6.48), Degrees.of(7));
      // Extrapolated with a linreg
      HOOD_ANGLE_BY_DISTANCE.put(Meters.of(20), Degrees.of(28.9));
    }

    public static final InterpolatingMeasureTreeMap<Distance, DistanceUnit, Time, TimeUnit> TIME_TO_SCORE_BY_DISTANCE = new InterpolatingMeasureTreeMap<>();

    /**
     * An interpolation table used for hood angle by gamepiece velocity.
     */
    public static final InterpolatingMeasureTreeMap<Angle, AngleUnit, Angle, AngleUnit> HOOD_ANGLE_BY_GAMEPIECE_THETA = new InterpolatingMeasureTreeMap<>();

    static {
      // Add values to the interpolation table
      HOOD_ANGLE_BY_GAMEPIECE_THETA.put(Degrees.of(37.0), Degrees.of(1.0));
      HOOD_ANGLE_BY_GAMEPIECE_THETA.put(Degrees.of(69.5), Degrees.of(32.0));
    }

    static {
      // for high arc
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(1.9), Seconds.of(0.77133478759));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(2.58), Seconds.of(1.02370063866));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(3.29), Seconds.of(1.05972742105));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(4.5), Seconds.of(1.12811589605));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(5.29), Seconds.of(1.36338398207));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(6.48), Seconds.of(1.71443901118));
      // TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(20), Seconds.of(3.98172596723));

      // for low arc
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(1.9), Seconds.of(0.604051441332));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(2.58), Seconds.of(0.572693676756));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(3.29), Seconds.of(0.678460601952));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(4.5), Seconds.of(0.845008647989));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(5.29), Seconds.of(0.813517027033));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(6.48), Seconds.of(0.785233152962));
      TIME_TO_SCORE_BY_DISTANCE.put(Meters.of(20), Seconds.of(1.02610888441));
    }

    /**
     * The angle to shoot the gamepiece at.
     */
    public static final Angle GAMEPIECE_THETA = Degrees.of(80.0);

    /**
     * The acceleration due to gravity imposed on the gamepiece.
     */
    public static final LinearAcceleration GAMEPIECE_G = MetersPerSecondPerSecond.of(-9.81);

    /**
     * How long to wait after no balls are detected to stop shooting.
     */
    public static final Time SHOOTING_TIMEOUT = Seconds.of(2.0);

    // TODO fill in once linreg is done
    /**
     * the A in the linreg for x=hood angle y = output angle
     */
    public static final double LINREG_HOOD_ANGLE_A = 0;
    /**
     * the B in the linreg for x=hood angle y = output angle
     */
    public static final double LINREG_HOOD_ANGLE_B = 0;

    /**
     * the A in the linreg for x=flywheel speed y = output speed
     */
    public static final double LINREG_FLYWHEEL_A = .225;
    /**
     * the B in the linreg for x=flywheel speed y = output speed
     */
    // currently forcing the intercept to 0, might change later
    public static final double LINREG_FLYWHEEL_B = 0;

    public static final int SHOOTER_CALCLATOR_SHOT_ITERATION = 3;

    /**
     * The minimum angle to shoot at.
     */
    public static final Angle MIN_SHOOT_ANGLE = Degrees.of(38);
    /**
     * The maximum angle to shoot at.
     */
    public static final Angle MAX_SHOOT_ANGLE = Degrees.of(69);

    /**
     * A set of values to shoot from a static position.
     */
    public static final ShooterValues STATIC_SHOOT_VALUES = new ShooterValues(RPM.of(2200), Degrees.of(0), Degrees.of(13));
    /**
     * A set of values to shoot from the left by the door.
     */
    public static final ShooterValues STATIC_SHOOT_LEFT_DOOR_VALUES = new ShooterValues(RotationsPerSecond.of(39), Degrees.of(0), Degrees.of(38));
    /**
     * A set of values to shoot from the right by the door.
     */
    public static final ShooterValues STATIC_SHOOT_RIGHT_DOOR_VALUES = new ShooterValues(RotationsPerSecond.of(38), Degrees.of(5), Degrees.of(-37));
    /**
     * A set of values to shoot from the center by the tower.
     */
    public static final ShooterValues STATIC_SHOOT_CENTER = new ShooterValues(RotationsPerSecond.of(33), Degrees.of(5), Degrees.of(0));
  }
}
