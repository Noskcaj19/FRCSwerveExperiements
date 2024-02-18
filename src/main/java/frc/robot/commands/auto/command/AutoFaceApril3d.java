package frc.robot.commands.auto.command;
// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.DriveSubsystem;

// public class AutoFaceApril3d extends Command {
//   private DriveSubsystem ds;
//   private ProfiledPIDController pidY = new ProfiledPIDController(
//       2,
//       0.0,
//       0,
//       new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxVelocityMetersPerSecond
//           / 4, 2.5));
//   private ProfiledPIDController pidX = new ProfiledPIDController(
//       .25,
//       0.0,
//       0,
//       new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond
//           / 4, .25));
//   private ProfiledPIDController pidTheta = new ProfiledPIDController(
//       .06,
//       0.0,
//       0,
//       new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond / 4, .5));

//   private static double[] getTagPose() {
//     var pose = LimelightHelpers.getTargetPose_RobotSpace("");
//     return pose;
//     // return new Pose3d(pose[0], -pose[1], pose[2], new Rotation3d(pose[3],
//     // pose[4], pose[5]));
//   }

//   public AutoFaceApril3d(DriveSubsystem ds) {
//     var tune = Shuffleboard.getTab("Tune");
//     tune.add("April PID X", pidX);
//     tune.add("April PID Y", pidY);
//     tune.add("April PID theta", pidTheta);

//     pidX.setGoal(new State(0, 0));
//     pidY.setGoal(new State(0.9, 0));
//     pidTheta.setGoal(new State(0, 0));

//     addRequirements(ds);
//     this.ds = ds;
//   }

//   @Override
//   public void execute() {
//     var p = getTagPose();

//     var x = 0.0;
//     x = pidX.calculate(p[0]);
//     // x = pidX.calculate(p.getTranslation().getX());
//     x = MathUtil.clamp(x, -.4, .4);

//     var y = 0.0;
//     // y = pidY.calculate(p[2]);
//     y = MathUtil.clamp(y, -.2, .2);

//     var theta = 0.0;
//     // theta = pidTheta.calculate(p[4]);
//     // System.out.printf("X%.2fY%.2fZ%.2f\n", p[3], p[4], p [5]);
//     theta = MathUtil.clamp(theta, -.1, .1);
//     System.out.printf("Y: %.2f t: %.2f\n", p[4], theta);

//     ds.drivePercent(0, 0, x, false, 0, 0);// p.getX(), p.getY());
//     // ds.drivePercent(-y, x, 0, false, 0,0);//p.getX(), p.getY());
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
