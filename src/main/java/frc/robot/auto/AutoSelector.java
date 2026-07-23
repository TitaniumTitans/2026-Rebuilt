package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
//  private final LoggedDashboardChooser<Command> chooser =
//      new LoggedDashboardChooser<>("AutoChooser");
  private final SendableChooser<Command> autoChooser;

  public AutoSelector(Drive drive) {
//    chooser.addDefaultOption("None", Commands.none());

    // Testing paths
//    chooser.addOption("Linear Test",
//        AutoBuilder.buildAuto("LinearTest"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected()
        .andThen(Commands.print("Command finished!"))
        .handleInterrupt(() ->
            CommandScheduler.getInstance().schedule(Commands.print("Interrupted!")));
  }
}