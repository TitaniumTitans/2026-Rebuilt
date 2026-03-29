package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoUtils;
import frc.robot.util.FieldConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

import static edu.wpi.first.units.Units.Volts;

public class AutoCommands {
  public static Command followChoreoPath(String name) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
  }

  public static Command resetPoseAndFollowChoreoPath(Drive drive, String name) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(ChoreoUtils.getPathStartingPose(name).getPose())),
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

  public static Command aimAndShoot(Drive swerve, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intake) {
    return DriveCommands.joystickDriveAtAngle(
            swerve,
            () -> 0.0,
            () -> 0.0,
            () -> RobotState.getInstance().getPointAtAngle(() -> AllianceFlipUtil.apply(FieldConstants.Hub.goalPoint))
        )
        .alongWith(shooterSubsystem.autoAim())
        .alongWith(
            Commands.waitUntil(shooterSubsystem::flywheelAtRPM)
                .andThen(feederSubsystem.runFeeder(Volts.of(12.0))
                    .alongWith(intake.agitateCommand())
                )
        ).withTimeout(7.0)
        .andThen(intake.setIntakePower(IntakeSubsystem.Speed.STOP));

  }

}