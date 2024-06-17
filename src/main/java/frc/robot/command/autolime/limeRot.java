package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsytems.SwerveSubsystem;

public class limeRot extends Command {

    private final PIDController turnPID;
    private SwerveSubsystem swerveSub;
    private LimelightHelpers limelightHelpers;

    public limeRot(SwerveSubsystem swerveSub, LimelightHelpers limelightHelpers) {
        addRequirements(swerveSub);

        this.swerveSub = swerveSub;
        this.limelightHelpers = limelightHelpers;
        turnPID = new PIDController(0.008, 0, 0);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("")) {
            double rotOut = turnPID.calculate(LimelightHelpers.getTX(""));

            rotOut = MathUtil.clamp(rotOut, -0.2, 0.2);
            swerveSub.drive(0, 0, rotOut, false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
