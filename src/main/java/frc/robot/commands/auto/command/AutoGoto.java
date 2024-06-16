package frc.robot.commands.auto.command;

import static frc.robot.Constants.DriveConstants.kMaxVelocityMetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoGoto extends Command {
    private DriveSubsystem ds;
    private Pose2d startingPose;
    private Translation2d movement;

    // private final PIDConstants translationConstraints = new PIDConstants(9.5,
    // 0.0, 0.0), // Translation PID constants
    // new PIDConstants(9, 0.0, 0.0), // Rotation PID constants

    private final double ktp = 4;
    private final double kti = 0;
    private final double ktd = 0;

    private final TrapezoidProfile.Constraints limits = new TrapezoidProfile.Constraints(
            Units.MetersPerSecond.of(DriveConstants.kMaxVelocityMetersPerSecond*.7),
            Units.MetersPerSecondPerSecond.of(6));
private final double ku = 7;
    private final double tu = 1.7;

    // private final ProfiledPIDController xPID = new ProfiledPIDController(
    //    1,
    //    0,
    //    0,
    //    limits);

    private final ProfiledPIDController xPID = new ProfiledPIDController(
      .6*ku,
      .5*tu,
      .125*tu,
       limits);
    private final ProfiledPIDController yPID = new ProfiledPIDController(
      .6*ku,
      .5*tu,
      .125*tu,
       limits);
//    private final ProfiledPIDController xPID = new ProfiledPIDController(ktp, kti, ktd, limits);
//    private final ProfiledPIDController yPID = new ProfiledPIDController(ktp, kti, ktd, limits);
    private double targetThresholdDistance = Units.Inches.of(3).in(Units.Meters);

    public AutoGoto(DriveSubsystem ds, Translation2d movement) {
        this.movement = movement;
        this.ds = ds;


        xPID.setGoal(movement.getX());
        yPID.setGoal(movement.getY());

        // Shuffleboard.getTab("Tune").add("AutoGoto xPID", xPID);
        // Shuffleboard.getTab("Tune").addDouble("AutoGoto xPID Goal", () -> xPID.getGoal().position);
        // Shuffleboard.getTab("Tune").addDouble("AutoGoto xPID Setpoint", () -> xPID.getSetpoint().position);

        
        addRequirements(ds);
    }

    @Override
    public void initialize() {
        startingPose = ds.getPose();
        xPID.reset(0);
        yPID.reset(0);
    }

    @Override
    public void execute() {
        var relativePose = ds.getPose().relativeTo(startingPose);

        // NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Debug/Pose").setValue(relativePose.getX());

        var xSpeed = xPID.calculate(relativePose.getX());
        var ySpeed = yPID.calculate(relativePose.getY());

        System.out.println("Rel: " + relativePose + ", goal: " + movement + " x: " + xSpeed + " y: " + ySpeed);

        xSpeed = MathUtil.clamp(xSpeed, -kMaxVelocityMetersPerSecond / 2, kMaxVelocityMetersPerSecond / 2);
        ySpeed = MathUtil.clamp(ySpeed, -kMaxVelocityMetersPerSecond / 2, kMaxVelocityMetersPerSecond / 2);

        ds.drivePercent(xSpeed / kMaxVelocityMetersPerSecond, ySpeed / kMaxVelocityMetersPerSecond, 0, false, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        ds.drivePercent(0, 0, 0, false, 0, 0);
    }

    @Override
    public boolean isFinished() {
        var dist = ds.getPose().relativeTo(startingPose).getTranslation().minus(movement).getNorm();
        // return false;
        System.out.println("done: " + (dist<targetThresholdDistance) + " : " + dist);
        

        return (dist < targetThresholdDistance);
    }

}
