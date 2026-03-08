package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("AutoChooser");

  public AutoSelector(DriveSubsystem drive) {
    chooser.addDefaultOption("None", Commands.none());

    // Testing paths
    chooser.addOption("Linear Test",
        AutoBuilder.buildAuto("LinearTest"));
  }

  public Command getAutoCommand() {
    return chooser.get();
  }
}