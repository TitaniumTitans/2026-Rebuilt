package frc.robot.commands;


import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class LeftCenterAutoCommandGroup extends SequentialCommandGroup {
  public LeftCenterAutoCommandGroup(Drive drive, IntakeSubsystem intake) {
    super(drive.followPathCustomCommand("LeftBumpSend", true)
            .alongWith(intake.homingCommand()),
        intake.setPivotPosition(IntakeSubsystem.Position.INTAKE),
//        Commands.waitSeconds(1.0),
        intake.setIntakePower(IntakeSubsystem.Speed.INTAKE),
        drive.followPathCommand("LeftSweepOne", false),
//        intake.setIntakePower(IntakeSubsystem.Speed.STOP),
//        Commands.waitSeconds(1.0),
        drive.followPathCustomCommand("LeftBumpReturn", false),
        NamedCommands.getCommand("aimAndShoot"));
  }
}