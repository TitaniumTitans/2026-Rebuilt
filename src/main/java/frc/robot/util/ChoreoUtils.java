package frc.robot.util;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class ChoreoUtils {

  public static AllianceFlipUtil.MaybeFlippedPose2d getPathStartingPose(String pathName) {
    Trajectory<?> trajectory = Choreo.loadTrajectory(pathName).orElseThrow();
    return new AllianceFlipUtil.MaybeFlippedPose2d(trajectory.getInitialPose(false).get(), trajectory.getInitialPose(true).get());
  }
}