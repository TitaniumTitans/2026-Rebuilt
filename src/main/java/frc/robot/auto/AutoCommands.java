package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.ChoreoUtils;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

public class AutoCommands {
  public static Command followChoreoPath(String name) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
  }

  public static Command resetPoseAndFollowChoreoPath(DriveSubsystem drive, String name) {
    return Commands.sequence(
        drive.resetPoseFactory(ChoreoUtils.getPathStartingPose(name).getPose()),
        followChoreoPath(name)
    );
  }

  public static Command lowerIntake(IntakeSubsystem intakeSubsystem) {
    return intakeSubsystem.setPivotPosition(IntakeSubsystem.Position.INTAKE);
  }

  public static Command runIntake(IntakeSubsystem intakeSubsystem) {
    return intakeSubsystem.setIntakePower(IntakeSubsystem.Speed.INTAKE);
  }

  public static Command stopIntake(IntakeSubsystem intakeSubsystem) {
    return intakeSubsystem.setIntakePower(IntakeSubsystem.Speed.STOP);
  }

  public static Command raiseIntake(IntakeSubsystem intakeSubsystem) {
    return intakeSubsystem.setPivotPosition(IntakeSubsystem.Position.STOWED);
  }
  

}