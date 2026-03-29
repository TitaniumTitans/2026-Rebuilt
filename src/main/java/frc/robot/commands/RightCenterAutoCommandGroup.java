package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RightCenterAutoCommandGroup extends SequentialCommandGroup {
  public RightCenterAutoCommandGroup(Drive drive, IntakeSubsystem intake) {
    // TODO: Add your sequential commands in the super() call, e.g.
    //           super(new OpenClawCommand(), new MoveArmCommand());
    super(drive.followPathCommand("RightBumpSend"),
        Commands.waitSeconds(1.0),
        drive.followPathCommand("RightSweepOne"),
        Commands.waitSeconds(1.0),
        drive.followPathCommand("RightBumpReturn"));
  }
}