package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.SwerveSubsystem;

public class DefaultSwerve extends Command {

    Boolean slow = false;
    private Joystick joy;
    private SwerveSubsystem swerveSub;
    public DefaultSwerve(Joystick joy, SwerveSubsystem swerveSub) {

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.joy = joy;
    }

    public double signedPow(double v, double p) {
        return Math.copySign(Math.pow(v, p), v);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // swerve stuff goes here
        // xspeed is xbox controller left joystick yspeed is also left joystick and
        // rotation is right joystick

        // adding deadbands


        var xSpeed = signedPow(MathUtil.applyDeadband(-joy.getY(), 0.1), 2);
        var ySpeed = signedPow(MathUtil.applyDeadband(-joy.getX(), 0.1), 2);
        var rot = signedPow(MathUtil.applyDeadband(-joy.getTwist(), 0.1), 3);

        if (!joy.getTrigger()) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;
            rot *= 0.4;
        } else {
            rot *= 0.5;
        }

        if (joy.getRawButton(12)) {
            swerveSub.setFodOffset();
            // swerveSub.zeroYaw();
            // swerveSub.resetOmetry(new Pose2d(1,1, new Rotation2d()));
        }

        swerveSub.drive(xSpeed, ySpeed, rot, !joy.getRawButton(2));

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
