package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoFaceApril3d extends Command {
  private DriveSubsystem ds;
  private ProfiledPIDController pidY = new ProfiledPIDController(
  2,
  0.0,
  0,
  new
  TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxVelocityMetersPerSecond
  / 4, 2.5));
  private ProfiledPIDController pidX = new ProfiledPIDController(
  2,
  0.0,
  0,
  new
  TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxVelocityMetersPerSecond
  / 4, 2.5));
  private ProfiledPIDController pidTheta = new ProfiledPIDController(
      .06,
      0.0,
      0,
      new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond / 4, 1.5));

  private static Pose3d getTagPose() {
    var pose = LimelightHelpers.getTargetPose_RobotSpace("");
    return new Pose3d(pose[0], -pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public AutoFaceApril3d(DriveSubsystem ds) {
    pidX.setGoal(new State(0, 0));
    pidY.setGoal(new State(0.9, 0));
    pidTheta.setGoal(new State(0, 0));

    addRequirements(ds);
    this.ds = ds;
  }

  @Override
  public void execute() {
    var p = getTagPose();
    System.out.println(p);

    var x = 0.0;
    x = pidX.calculate(p.getTranslation().getX());
    x = MathUtil.clamp(x, -.6, .6);

    var y = 0.0;
    y = pidY.calculate(p.getTranslation().getZ());
    y = MathUtil.clamp(y, -.2, .2);

    var theta = 0.0;
    theta = pidTheta.calculate(p.getRotation().getZ());
    theta = MathUtil.clamp(theta, -.1, .1);

    ds.drivePercent(-y, x, -theta, false, 0,0);//p.getX(), p.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
