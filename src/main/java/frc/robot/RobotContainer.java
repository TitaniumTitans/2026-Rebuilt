// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.gos.lib.properties.PropertyManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;


public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);

    private final DriveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;

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

              shooter = new ShooterSubsystem(new ShooterIOTalonFX());
              intake = new IntakeSubsystem(new IntakeIOTalonFX());
              feeder = new FeederSubsystem(new FeederIOTalonFX());
          }
          case SIM -> {
              swerve = new DriveSubsystem(
                  new GyroIOSim(),
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );

              shooter = new ShooterSubsystem(new ShooterIO() {});
              intake = new IntakeSubsystem(new IntakeIO() {});
              feeder = new FeederSubsystem(new FeederIO() {});
          }
          case REPLAY -> {
              swerve = new DriveSubsystem(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );

              shooter = new ShooterSubsystem(new ShooterIO() {});
              intake = new IntakeSubsystem(new IntakeIO() {});
              feeder = new FeederSubsystem(new FeederIO() {});
          }
          default -> throw new IllegalStateException("Unexpected value: " + Constants.getMode());
        }

        configureBindings();
        configureDashboardCommands();
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

      driveController.povUp().onTrue(shooter.setHoodPosition(0.3));
      driveController.povDown().onTrue(shooter.setHoodPosition(0.1));

      driveController.a().whileTrue(shooter.runDashboardRPM());
      driveController.b().whileTrue(feeder.runFeeder(Volts.of(12.0)));

      driveController.x().onTrue(intake.homingCommand());

      driveController.rightBumper().whileTrue(intake.intake());
      driveController.leftBumper().onTrue(intake.setPivotPosition(IntakeSubsystem.Position.STOWED));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }

    public void configureDashboardCommands() {
        // manually control the pivot hood
        SmartDashboard.putData("Hood to %10", shooter.setHoodPosition(0.1));
        SmartDashboard.putData("Hood to %30", shooter.setHoodPosition(0.3));
        SmartDashboard.putData("Hood to %50", shooter.setHoodPosition(0.5));
        SmartDashboard.putData("Hood to %70", shooter.setHoodPosition(0.7));
        SmartDashboard.putData("Hood to %90", shooter.setHoodPosition(0.9));

        SmartDashboard.putData("Reset Field Oriented", Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d())).withName("Reset Field Orient"));
    }
}
