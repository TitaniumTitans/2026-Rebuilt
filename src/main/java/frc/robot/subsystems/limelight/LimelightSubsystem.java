package frc.robot.subsystems.limelight;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LimelightSubsystem extends SubsystemBase {
  private final String name;

  public LimelightSubsystem(String name) {
    this.name = name;
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
    LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    final LimelightHelpers.PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    final LimelightHelpers.PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (
        poseEstimate_MegaTag1 == null
            || poseEstimate_MegaTag2 == null
            || poseEstimate_MegaTag1.tagCount == 0
            || poseEstimate_MegaTag2.tagCount == 0
    ) {
      return Optional.empty();
    }

    // Combine the readings from MegaTag1 and MegaTag2:
    // 1. Use the more stable position from MegaTag2
    // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
    poseEstimate_MegaTag2.pose = new Pose2d(
        poseEstimate_MegaTag2.pose.getTranslation(),
        poseEstimate_MegaTag1.pose.getRotation()
    );
    final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 1.0);

    Logger.recordOutput("Limelight/" + name + "/poseEstimate", poseEstimate_MegaTag2.pose);

    return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
  }

  public Optional<Double> getHumanAimbot() {
    Optional<LimelightHelpers.RawDetection> detection =  Arrays.stream(LimelightHelpers.getRawDetections(name))
        .filter(match -> match.classId == 0)
        .findFirst();

    return detection.map(rawDetection -> rawDetection.txnc);
  }

  public static class Measurement {
    public final LimelightHelpers.PoseEstimate poseEstimate;
    public final Matrix<N3, N1> standardDeviations;

    public Measurement(LimelightHelpers.PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
      this.poseEstimate = poseEstimate;
      this.standardDeviations = standardDeviations;
    }
  }
}