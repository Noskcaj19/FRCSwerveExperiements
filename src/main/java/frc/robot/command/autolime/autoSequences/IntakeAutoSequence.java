package frc.robot.command.autolime.autoSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.command.autolime.AutoAlignNotes;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class IntakeAutoSequence extends SequentialCommandGroup {

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;

    public IntakeAutoSequence(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;
        addCommands(
                // new AutoShoot(shooterSub, intakeSub).withTimeout(3),
                // new AutoDrive(swerveSub, 1, 0.2).withTimeout(2),
                // new AutoRotate(swerveSub, -10, 0.1).withTimeout(3),
                // new AutoIntakeDrive(swerveSub, intakeSub, 0.5, 0.3),
                // new AutoRotate(swerveSub, 10, 0.1),
                // new AutoDrive(swerveSub, 1, 0.2).withTimeout(2),
                // new AutoAlignTags(swerveSub, 1)
                // new AutoIntake(in)
                Commands.race(
                        new AutoAlignNotes(swerveSub).withTimeout(5),
                        Commands.race(
                                new AutoIntake(intakeSub),
                                new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.3))
                        )
                )
        );
    }
}

