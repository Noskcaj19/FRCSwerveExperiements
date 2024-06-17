package frc.robot.command.autolime;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.SwerveSubsystem;

public class AutoIntakeDrive extends Command {
    private SwerveSubsystem swerveSub;
    private Intake intakeSub;
    private Translation2d startPos;
    private double driveSpeed;
    private double goalDist;
    private double curDist;

    public AutoIntakeDrive(SwerveSubsystem swerveSub, Intake intakeSub, double goalDistance, double driveSpeed) {
        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {
        startPos = swerveSub.getPose().getTranslation();
    }

    @Override
    public void execute() {
        intakeSub.smartIntake();
        swerveSub.drive(driveSpeed, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stopSmIntake();
        swerveSub.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {

        // TODO tweak for going backwards?

        curDist = swerveSub.getPose().getTranslation().getDistance(startPos);
        if (goalDist < curDist) {
            return true;
        } else {
            return false;
        }
        // i finally get it now
        // goal distance is how far we want the robot to go
        // dist is how far we currently are away from the starting pos
        // if how far away we currently are is greater than how far away we want to be
        // the auto is finished
        // :DDDD
    }


}
