package frc.robot.subsystems.vision;

import com.gos.lib.properties.TunableTransform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import org.photonvision.simulation.SimCameraProperties;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public class VisionConstants {
  public static final VisionFilterParameters FILTER_PARAMETERS = new VisionFilterParameters(
      0.05, // 0.5, x-y standard deviation
      0.15, // 1.5, rotation standard deviation
      Units.Centimeters.of(21), // width of the april tag
      0.5, // max ambiguity
      Rotation2d.fromDegrees(92), // camera FOV
      Units.Centimeters.of(805), //field width
      Units.Centimeters.of(1755), // field height
      Units.Centimeters.of(35)); // 15, z off the ground

  public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();

  static {
    SIM_CAMERA_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(70));
    SIM_CAMERA_PROPERTIES.setCalibError(0.52, 0.08);
    SIM_CAMERA_PROPERTIES.setFPS(40);
    SIM_CAMERA_PROPERTIES.setAvgLatencyMs(20);
    SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(10);
  }

//  public static final TunableTransform3d SIM_CAMERA_TRANSFORM = new TunableTransform3d(
//      false,
//      "CameraTransforms/BackCamera/",
//      new Transform3d(
//          new Translation3d(),
//          new Rotation3d()
//      )
//  );

  public static final TunableTransform3d RIGHT_CAMERA_TRANSFORM = new TunableTransform3d(
      false,
      "CameraTransforms/ShooterRightCamera/t",
      new Transform3d(
        new Translation3d(),
        new Rotation3d()
      )
  );

  public static final TunableTransform3d LEFT_CAMERA_TRANSFORM = new TunableTransform3d(
      false,
      "CameraTransforms/ShooterLeftCamera/t",
      new Transform3d(
          new Translation3d(),
          new Rotation3d()
      )
  );
}
