// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;


public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);

    private final DriveSubsystem swerve;

    public RobotContainer()
    {
        switch (Constants.getMode()) {
          case REAL -> {
              swerve = new DriveSubsystem(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3])
              );
          }
          case SIM -> {
              swerve = new DriveSubsystem(
                  new GyroIOSim(),
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );
          }
          case REPLAY -> {
              swerve = new DriveSubsystem(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );
          }
          default -> throw new IllegalStateException("Unexpected value: " + Constants.getMode());
        }

        configureBindings();
    }
    
    
    private void configureBindings() {
      swerve.setDefaultCommand(
          DriveCommands.joystickDrive(
            swerve,
            () -> driveController.getLeftY(),
            () -> driveController.getLeftX(),
            () -> -driveController.getRightX()
          )
      );

      driveController.start().onTrue(
          Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d()))
      );
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
