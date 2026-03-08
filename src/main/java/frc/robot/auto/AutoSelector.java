package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.AllianceFlipUtil;
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