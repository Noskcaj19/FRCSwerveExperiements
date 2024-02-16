package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kMaxVelocityMetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoFaceApril extends Command {
  private DriveSubsystem ds;
//ku = 3
// Tu = .8
  double ku = 3.0;
  double tu = 0.8;
  private ProfiledPIDController pidT = new ProfiledPIDController(
      .6*ku,
      .5*tu,
      .125*tu,
      new TrapezoidProfile.Constraints(5, 3/1.5  ));

  private static double[] getTagPose() {
    var pose = LimelightHelpers.getTargetPose_RobotSpace("");
    return pose;
  }

  public AutoFaceApril(DriveSubsystem ds) {
    var tune = Shuffleboard.getTab("Tune");
    tune.add("April3 PID T", pidT);

    pidT.setGoal(new State(0, 0));

    addRequirements(ds);
    this.ds = ds;
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("")) {
      ds.drivePercent(0, 0, 0, false, 0, 0);
    }
    var p = getTagPose();

    var t = 0.0;
    t = pidT.calculate(p[0]);
    t = MathUtil.clamp(t, -kMaxVelocityMetersPerSecond/2, kMaxVelocityMetersPerSecond/2);

    // System.out.printf("Y: %.2f t: %.2f\n", p[4], );
    var def = NetworkTableInstance.getDefault();
    def.getEntry("Shuffleboard/Tune/turn").setDouble(t);

    ds.drivePercent(0, 0, t, false, 0, 0);;
    // ds.drivePercent(-y, x, 0, false, 0,0);//p.getX(), p.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
