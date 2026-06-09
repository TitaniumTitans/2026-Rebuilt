package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.LimelightSubsystem;


public class HumanAimbotCommand extends Command {
  private final Drive drive;
  private final LimelightSubsystem limelightSubsystem;

  private static final double ANGLE_KP = 0.125;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 0.5; // 8.0
  private static final double ANGLE_MAX_ACCELERATION = 0.125; // 20.0

  private static final PIDController TURNING_CONTROLLER = new PIDController(
      ANGLE_KP,
      0.0,
      ANGLE_KD);

  public HumanAimbotCommand(Drive drive, LimelightSubsystem limelightSubsystem) {
    this.drive = drive;
    this.limelightSubsystem = limelightSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive, this.limelightSubsystem);
  }

  @Override
  public void initialize() {
      limelightSubsystem.setPipeline(1);
  }

  @Override
  public void execute() {
    double output = TURNING_CONTROLLER.calculate(
        limelightSubsystem.getHumanAimbot().orElse(0.0), 0.0);

    output = MathUtil.clamp(output, -0.45, 0.45);

    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, output));
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.setPipeline(0);
  }
}
