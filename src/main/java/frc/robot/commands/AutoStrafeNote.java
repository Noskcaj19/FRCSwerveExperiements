package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kMaxVelocityMetersPerSecond;

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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoStrafeNote extends Command {
  private DriveSubsystem ds;
//ku = 3
// Tu = .8
  double ku = 3.0;
  double tu = 0.8;
  private ProfiledPIDController pidY = new ProfiledPIDController(
      .6*ku,
      .5*tu,
      .125*tu,
      new TrapezoidProfile.Constraints(5, 3/1.5));
  private ProfiledPIDController pidX = new ProfiledPIDController(
      .6*ku,
      .5*tu,
      .125*tu,
      new TrapezoidProfile.Constraints(5, 3/1.5  ));

  private static double[] getTagPose() {
    var pose = LimelightHelpers.getTargetPose_RobotSpace("");
    return pose;
  }

  public AutoStrafeNote(DriveSubsystem ds) {
    var tune = Shuffleboard.getTab("Tune");
    tune.add("April2 PID X", pidX);
    tune.add("April2 PID Y", pidY);

    pidX.setGoal(new State(0, 0));
    pidY.setGoal(new State(2, 0));

    addRequirements(ds);
    this.ds = ds;
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("")) {
      ds.drivePercent(0, 0, 0, false, 0, 0);
    }
    var p = getTagPose();

    var x = 0.0;
    x = pidX.calculate(p[0]);
    x = MathUtil.clamp(x, -kMaxVelocityMetersPerSecond/2, kMaxVelocityMetersPerSecond/2);

    var y = 0.0;
    y = pidY.calculate(p[2]);
    y = MathUtil.clamp(y, -kMaxVelocityMetersPerSecond/2, kMaxVelocityMetersPerSecond/2);

    // System.out.printf("Y: %.2f t: %.2f\n", p[4], );
    var def = NetworkTableInstance.getDefault();
    def.getEntry("Shuffleboard/Tune/strafex").setDouble(x);
    def.getEntry("Shuffleboard/Tune/strafey").setDouble(y);

    ds.drivePercent(-(y/kMaxVelocityMetersPerSecond), x/kMaxVelocityMetersPerSecond, 0, false, 0, 0);// p.getX(), p.getY());
    // ds.drivePercent(-y, x, 0, false, 0,0);//p.getX(), p.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
