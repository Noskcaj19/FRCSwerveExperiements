package frc.robot.command.autolime.autoSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.command.autolime.*;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class RightAuto extends SequentialCommandGroup {

    SwerveSubsystem swerveSub;
    Shooter ShooterSub;
    Intake intakeSub;

    public RightAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addCommands(
                new AutoDrive(swerveSub, 0.5, 0.5).withTimeout(2),
                new AutoAlignTags(swerveSub).withTimeout(0.5),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(3),
                new AutoDrive(swerveSub, 0.3, 0.6).withTimeout(1),
                new AutoRotate(swerveSub, -45, 0.3).withTimeout(3),
                //new AutoIntakeDrive(swerveSub, intakeSub, 5, 0.3).withTimeout(2)
                Commands.race(
                        new AutoDriveAndTrackNote(swerveSub, 2.5, 0.2),
                        Commands.race(
                                new AutoIntake(intakeSub),
                                new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.15))
                        )
                ),
                new AutoDrive(swerveSub, 1, 0.3).withTimeout(3),
                new AutoRotate(swerveSub, -30, 0.2),
                new AutoAlignTags(swerveSub).withTimeout(2),
                new AutoShootSmart(shooterSub, intakeSub)
        );
    }

}
