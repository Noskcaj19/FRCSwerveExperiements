package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsytems.SwerveSubsystem;

public class NoteRotationAlign extends Command {
    static int count = 0;
    double ku = .4;
    double tu = 1.1;
    double kuh = .05;
    double tuh = .7;
    MedianFilter horizontalFilter = new MedianFilter(3);
    SlewRateLimiter turnLimit = new SlewRateLimiter(1);
    int time = 0;
    private SwerveSubsystem swerveSub;
    private ProfiledPIDController rotationPid;

    public NoteRotationAlign(SwerveSubsystem swerveSub) {
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
        rotationPid = new ProfiledPIDController(0.03, 0, 0,
                // horizontalPid = new ProfiledPIDController(kuh*.6, 0, tuh*.125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        // Shuffleboard.getTab("pid_debug").add("note_rotation_PID"+count, rotationPid);
        count += 1;
    }

    final double getHorizontalOffset() {
        return horizontalFilter.calculate(LimelightHelpers.getTX("limelight-front"));
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }

    @Override
    public void initialize() {
        time = 0;
        // forwardPid.reset(1.5);
        // horizontalPid.reset(0);

    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-front")) {
            time += 1;
            var rot = rotationPid.calculate(getHorizontalOffset());
            rot = MathUtil.clamp(rot, -DriveConstants.MaxVelocityMetersPerSecond / 5, DriveConstants.MaxVelocityMetersPerSecond / 5);

            swerveSub.drive(0, 0, turnLimit.calculate(rot / DriveConstants.MaxAngularVelocityRadiansPerSecond), false);
        } else {
            if (time < 10) {
                swerveSub.drive(0, 0, -0.08, false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }

}