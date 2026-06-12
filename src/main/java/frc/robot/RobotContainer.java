// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoCommands;
import frc.robot.commands.*;
import frc.robot.generated.ChoreoVars;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.HubShiftUtil;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

/**
 * Container for robot subsystems, commands, and button bindings.
 * Instantiates subsystems based on robot mode (REAL/SIM/REPLAY).
 */
public class RobotContainer {
  // Driver controller
  private final CommandXboxController driveController = new CommandXboxController(0);

    private final Drive swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final VisionSubsystem vision;
//    private final LimelightSubsystem frontLimelight = new LimelightSubsystem("limelight-front");
    private final SendableChooser<Command> autoSelector;

    public RobotContainer()
    {
        switch (Constants.getMode()) {
          case REAL -> {
              swerve = new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight)
              );

              shooter = new ShooterSubsystem(new ShooterIOTalonFX());
              intake = new IntakeSubsystem(new IntakeIOTalonFX());
              feeder = new FeederSubsystem(new FeederIOTalonFX());
              vision = new VisionSubsystem(VisionConstants.FILTER_PARAMETERS,
                  new VisionIOPhotonReal(
                      "LeftCamera",
                      VisionConstants.LEFT_CAMERA_TRANSFORM,
                      FieldConstants.defaultAprilTagType.getLayout()),
                  new VisionIOPhotonReal(
                      "RightCamera",
                      VisionConstants.RIGHT_CAMERA_TRANSFORM,
                      FieldConstants.defaultAprilTagType.getLayout())
              );
          }
          case SIM -> {
              swerve = new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight)
              );

              shooter = new ShooterSubsystem(new ShooterIO() {});
              intake = new IntakeSubsystem(new IntakeIO() {});
              feeder = new FeederSubsystem(new FeederIO() {});
              vision = new VisionSubsystem(VisionConstants.FILTER_PARAMETERS,
                  new VisionIOPhotonSimulation(
                      "LeftCamera",
                      VisionConstants.LEFT_CAMERA_TRANSFORM,
                      FieldConstants.defaultAprilTagType.getLayout(),
                      VisionConstants.SIM_CAMERA_PROPERTIES));
          }
          case REPLAY -> {
              swerve = new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );

        shooter = new ShooterSubsystem(new ShooterIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        vision = new VisionSubsystem(VisionConstants.FILTER_PARAMETERS, new VisionIO() {});
      }
      default -> throw new IllegalStateException("Unexpected value: " + Constants.getMode());
    }

    configureBindings();
    configureDashboardCommands();
    configureNameCommands();

    boolean isComp = true;

    autoSelector = AutoBuilder.buildAutoChooserWithOptionsModifier(stream -> isComp
        ? stream.filter(auto -> auto.getName().endsWith("Comp"))
        : stream);
//      autoSelector = new SendableChooser<>();

//    autoSelector.addOption("Wheel Radius Characterization", swerve);

//      autoSelector.addOption("Wheel Radius Characterization",
//          DriveCommands.wheelRadiusCharacterization(swerve));
//      autoSelector.addOption("Simple FF Characterization",
//          DriveCommands.feedforwardCharacterization(swerve));
//      autoSelector.addOption("CustomLinearTest", swerve.followPathCommand("LinearPath"));
//      autoSelector.addOption("CustomRightBumpTest", swerve.followPathCustomCommand("RightBumpSend"));
      autoSelector.addOption("RightSweep", swerve.followPathCommand("RightCenter", true));
      autoSelector.addOption("RightCenter", new RightCenterAutoCommandGroup(swerve, intake, shooter, feeder));
      autoSelector.addOption("LeftCenter", new LeftCenterAutoCommandGroup(swerve, intake, shooter, feeder));
      autoSelector.addOption("RightCenterCC", new RightCenterAutoCCCommandGroup(swerve, intake, shooter, feeder));
      autoSelector.addOption("LeftCenterCC", new LeftCenterAutoCCCommandGroup(swerve, intake, shooter, feeder));

    SmartDashboard.putData("Auto Chooser", autoSelector);
  }

  /**
   * Configures driver controller button bindings.
   */
  private void configureBindings() {
    // Reset hub shift timer when enabling
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));

//    frontLimelight.setDefaultCommand(updateVisionCommand());

    //

//    matchTimeTrigger
//        .whileTrue(Commands.runOnce(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0))
//            .andThen(Commands.print("Rumble!")))
//        .whileFalse(Commands.runOnce(() -> driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
//            .andThen(Commands.print(" No Rumble 3:")));

    // Default drive command: joystick control
    swerve.setDefaultCommand(
      DriveCommands.joystickDrive(
        swerve,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> -driveController.getRightX()
      )
    );

    // Default vision command: feed vision updates to pose estimator
    vision.setDefaultCommand(
        vision.processVision(RobotState.getInstance()::getEstimatedPose)
            .ignoringDisable(true)
    );

    shooter.setDefaultCommand(shooter.shooterAutoHood());

    // Start button: reset robot pose to origin
    driveController.start().onTrue(
      Commands.runOnce(() -> swerve.setPose(new Pose2d()))
    );

//      driveController.povUp().onTrue(shooter.setHoodPosition(0.3));
//      driveController.povDown().onTrue(shooter.setHoodPosition(0.1));

    // A button: run shooter at dashboard RPM
    driveController.a().whileTrue(
        shooter.setHoodPosition(0.8)
            .andThen(shooter.runShooterRPM(RPM.of(3750)))
    ).onFalse(
        shooter.setHoodPosition(0.0)
    );

    // B button: run feeder at 12V
    driveController.b().whileTrue(feeder.runFeeder(Volts.of(12.0)));

    // X button: home the intake
    driveController.leftStick().onTrue(intake.homingCommand());

    // Y button: agitate intake
    driveController.y().whileTrue(intake.agitateCommand());

    // Right bumper: deploy and run intake
    driveController.rightBumper().whileTrue(intake.intake());

    // Left bumper: stow intake
    driveController.leftBumper().onTrue(intake.setPivotPosition(IntakeSubsystem.Position.STOWED));

    // Right trigger: auto aim
    driveController.rightTrigger().and(driveController.leftTrigger().negate()).whileTrue(
        DriveCommands.joystickDriveAtAngle(
            swerve,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> RobotState.getInstance().getPointAtAngle(() -> AllianceFlipUtil.apply(FieldConstants.Hub.goalPoint))
        ).alongWith(shooter.autoAim())
    );

    // Left Trigger: static shot
    driveController.x().whileTrue(
        shooter.setHoodPosition(0.1)
            .andThen(shooter.runDashboardRPM())
    );

    // force feed
    driveController.b().and(driveController.rightTrigger().or(driveController.a()).or(driveController.x()))
        .whileTrue(intake.agitateCommand());

    driveController.leftTrigger().and(driveController.rightTrigger())
        .whileTrue(Commands.run(swerve::stopWithX).alongWith(shooter.autoAim()));

//    driveController.leftTrigger()
//        .whileTrue(new HumanAimbotCommand(swerve, frontLimelight));

    // Right trigger: shoot on move
//    driveController.rightTrigger().whileTrue(
//        DriveCommands.joystickDriveAtAngle(
//            swerve,
//            () -> -driveController.getLeftY(),
//            () -> -driveController.getLeftX(),
//            () -> RobotState.getInstance().getShootOnMoveShotData().shotAngle()
//        ).alongWith(shooter.autoAimOnMove())
//    );

    driveController.povDown().whileTrue(
        swerve.driveToPose(() -> AllianceFlipUtil.apply(ChoreoVars.Poses.AltRightStart))
    );

    driveController.povUp().whileTrue(
        swerve.driveToPose(() -> AllianceFlipUtil.apply(ChoreoVars.Poses.CenterStart))
    );
    driveController.povRight().whileTrue(
        swerve.driveToPose(() -> AllianceFlipUtil.apply(ChoreoVars.Poses.RightStart))
    );
    driveController.povLeft().whileTrue(
        swerve.driveToPose(() -> AllianceFlipUtil.apply(ChoreoVars.Poses.AltLeftStart))
    );

//    driveController.povLeft().whileTrue(
//        swerve.driveToPose(
//            () -> AllianceFlipUtil.apply(new Pose2d(2.5, 5, new Rotation2d()))
//        )
//    );

//    driveController.povDown().whileTrue(
//        feeder.runFeeder(Volts.of(-12.0))
//            .alongWith(shooter.runShooterRPM(RPM.of(-3000)))
//    );
  }

  /**
   * Returns the autonomous command.
   * Currently configured to home the intake.
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  /**
   * Adds manual control commands to the SmartDashboard for testing.
   */
  public void configureDashboardCommands() {
    // Manual hood position control
    SmartDashboard.putData("Hood to %10", shooter.setHoodPosition(0.1));
    SmartDashboard.putData("Hood to %30", shooter.setHoodPosition(0.3));
    SmartDashboard.putData("Hood to %50", shooter.setHoodPosition(0.5));
    SmartDashboard.putData("Hood to %70", shooter.setHoodPosition(0.7));
    SmartDashboard.putData("Hood to %90", shooter.setHoodPosition(0.9));

    // Reset field-oriented driving
    SmartDashboard.putData(
      "Reset Field Oriented",
      Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d()))
        .withName("Reset Field Orient")
    );

    // Run current auto
    SmartDashboard.putData("Current Auto",
        AutoCommands.resetPoseAndFollowChoreoPath(swerve, "LinearTest")
        );

    SmartDashboard.putData("Clear Bump?",
        Commands.run(() -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(swerve.getMaxLinearSpeedMetersPerSec() * 0.75, 0, 0);
          swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getRotation()));
        }).withTimeout(1.25)
    );
  }

  public void configureNameCommands() {
    NamedCommands.registerCommand("homeIntake", intake.homingCommand());
    NamedCommands.registerCommand("lowerIntake", AutoCommands.lowerIntake(intake));
    NamedCommands.registerCommand("runIntake", AutoCommands.runIntake(intake));
    NamedCommands.registerCommand("stopIntake", AutoCommands.stopIntake(intake));
    NamedCommands.registerCommand("raiseIntake", AutoCommands.raiseIntake(intake));
    NamedCommands.registerCommand("aimAndShoot", AutoCommands.aimAndShoot(swerve, shooter, feeder, intake));

    NamedCommands.registerCommand("clearBump",
        Commands.run(() -> {
                  ChassisSpeeds speeds =
                      new ChassisSpeeds(swerve.getMaxLinearSpeedMetersPerSec() * 0.75, 0, 0);
                  swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getRotation()));
            }
            )
            .withTimeout(1.5)
    );

    NamedCommands.registerCommand("resetRight",
        Commands.runOnce(
            () ->RobotState.getInstance().resetPose(
                () -> AllianceFlipUtil.apply(ChoreoVars.Poses.RightStart)
            )
        )
    );

    NamedCommands.registerCommand("resetLeft",
        Commands.runOnce(
            () ->RobotState.getInstance().resetPose(
                () -> AllianceFlipUtil.apply(ChoreoVars.Poses.LeftStart)
            )
        )
    );

    NamedCommands.registerCommand("spinupFlywheel", shooter.spinupFlywheel());
  }

  private boolean isTimeNear(double seconds) {
    return (seconds + 3) > DriverStation.getMatchTime() && DriverStation.getMatchTime() < (seconds - 3);
  }

  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
  }

//  private Command updateVisionCommand() {
//    return frontLimelight.run(() -> {
//          final Pose2d currentRobotPose = RobotState.getInstance().getEstimatedPose();
//          final Optional<LimelightSubsystem.Measurement> measurement = frontLimelight.getMeasurement(currentRobotPose);
//          measurement.ifPresent(m -> {
//            RobotState.getInstance().addVisionMeasurement(
//                new RobotState.VisionObservation(
//                m.poseEstimate.pose,
//                m.poseEstimate.timestampSeconds,
//                m.standardDeviations
//                )
//            );
//          });
//        })
//        .ignoringDisable(true);
//  }
}
