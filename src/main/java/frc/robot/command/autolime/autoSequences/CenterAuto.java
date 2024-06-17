package frc.robot.command.autolime.autoSequences;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.StopCommand;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoDrive;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class CenterAuto extends SequentialCommandGroup {

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;
    private MedianFilter zFilter = new MedianFilter(7);


    public CenterAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        addRequirements(intakeSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        var autoAlign = new AutoAlignTags(swerveSub);

        addCommands(
                // new AutoShoot(shooterSub, intakeSub).until(intakeSub::doesntHaveNote).withTimeout(2),
                new AutoAlignTags(swerveSub).withTimeout(.5),
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub),
                Commands.race(
                        new AutoDrive(swerveSub, 3, 0.2),
                        Commands.race(
                                new AutoIntake(intakeSub),
                                new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.3))
                        )
                ),
                new AutoDrive(swerveSub, 1, -0.2).until(this::closeEnough).withTimeout(4),
                autoAlign.until(autoAlign::aligned),//.until(AutoAlignTags::aligned),
                new AutoAlignTags(swerveSub).withTimeout(.5),
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(4),
                new AutoDrive(swerveSub, 2.5, 0.2).withTimeout(4)
        );
    }

    private boolean closeEnough() {
        if (LimelightHelpers.getTV("limelight-back")) {
            var rawZ = LimelightHelpers.getTargetPose3d_CameraSpace(("limelight-back")).getZ();
            var z = zFilter.calculate(rawZ);
            if (z < 1.3) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }

    }
}