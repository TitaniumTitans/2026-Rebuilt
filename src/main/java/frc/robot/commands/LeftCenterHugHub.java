package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class LeftCenterHugHub extends SequentialCommandGroup {
  public LeftCenterHugHub(Drive drive, IntakeSubsystem intake, ShooterSubsystem shooter, FeederSubsystem feeder) {
    super(
        drive.followPathCustomCommand("LeftBumpSend", true)
            .alongWith(intake.homingCommand()),
        drive.followPathCommand("LeftHugHub", false),
            Commands.waitSeconds(4.0),
        intake.setPivotPosition(IntakeSubsystem.Position.INTAKE),
//        Commands.waitSeconds(1.0),
        intake.setIntakePower(IntakeSubsystem.Speed.INTAKE),
        drive.followPathCommand("LeftHugReturn", false),
//        intake.setIntakePower(IntakeSubsystem.Speed.STOP),
//        Commands.waitSeconds(1.0),
        drive.followPathCommand("LeftBumpReturn", false),
        AutoCommands.aimAndShoot(drive, shooter, feeder, intake)
            .withTimeout(8.0)
            .withName("AutoShooting"),
        drive.followPathCustomCommand("LeftBumpSendTwo", false));
  }
}