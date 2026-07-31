package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RightCenterAutoCommandGroup extends SequentialCommandGroup {
  public RightCenterAutoCommandGroup(Drive drive, IntakeSubsystem intake, ShooterSubsystem shooter, FeederSubsystem feeder) {
    super(
        drive.followPathCustomCommand("RightBumpSend", true)
            .alongWith(intake.homingCommand()),
        intake.setPivotPosition(IntakeSubsystem.Position.INTAKE),
//        Commands.waitSeconds(1.0),
        intake.setIntakePower(IntakeSubsystem.Speed.INTAKE),
        drive.followPathCommand("RightSweepOne", false),
//        intake.setIntakePower(IntakeSubsystem.Speed.STOP),
//        Commands.waitSeconds(1.0),
        drive.followPathCommand("RightBumpReturn", false),
        AutoCommands.aimAndShoot(drive, shooter, feeder, intake)
            .withTimeout(8.0)
            .withName("AutoShooting"),
        drive.followPathCustomCommand("RightBumpSendTwo", false));
  }
}