package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsytems.SwerveSubsystem;

public class PIDAutodrive extends Command {

    private SwerveSubsystem swerveSub;
    private Translation2d point;
    private Pose2d startPosition;
    private ProfiledPIDController xPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2));
    private ProfiledPIDController yPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2));

    public PIDAutodrive(SwerveSubsystem swerveSub, Translation2d point) {

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.point = point;

        xPID.setGoal(point.getX());
        yPID.setGoal(point.getY());

    }

    @Override
    public void initialize() {
        startPosition = swerveSub.getPose();
    }

    @Override
    public void execute() {
        var relativePose = swerveSub.getPose().relativeTo(startPosition);

        var xSet = xPID.calculate(relativePose.getX());
        var ySet = yPID.calculate(relativePose.getY());

        xSet = MathUtil.clamp(xSet, -Constants.DriveConstants.MaxVelocityMetersPerSecond / 2, Constants.DriveConstants.MaxVelocityMetersPerSecond / 2);
        ySet = MathUtil.clamp(ySet, -Constants.DriveConstants.MaxVelocityMetersPerSecond / 2, Constants.DriveConstants.MaxVelocityMetersPerSecond / 2);

        swerveSub.drive(xSet / Constants.DriveConstants.MaxVelocityMetersPerSecond, ySet / Constants.DriveConstants.MaxVelocityMetersPerSecond, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }

    // @Override
    // public boolean isFinished() {

    //     var dist = swerveSub.getPose().relativeTo(startPosition).getTranslation().minus(point).getDistance(point);

    //     if( > dist)
    //     return false;
    // }

}//idk anymore