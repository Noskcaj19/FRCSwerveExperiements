package frc.robot.command.autolime.autoSequences;

import java.util.Optional;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.StopCommand;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoDrive;
import frc.robot.command.autolime.AutoDriveAndTrackNote;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoRotate;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.command.autolime.NoteRotationAlign;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class ThreeNoteCenterAuto extends SequentialCommandGroup{

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;

    public ThreeNoteCenterAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        addRequirements(intakeSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        var autoAlign = new AutoAlignTags(swerveSub);
        var autoAlignSpeaker3 = new AutoAlignTags(swerveSub);
        
        

        
        addCommands(
                new AutoAlignTags(swerveSub).withTimeout(.5),
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub),
                Commands.race(
                    new AutoDriveAndTrackNote(swerveSub, 3, 0.2),
                    Commands.race(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.15))
                    )
                ),
                new AutoDrive(swerveSub, 1, -0.2).until(this::closeEnough).withTimeout(4),
                autoAlign.until(autoAlign::aligned),//.until(AutoAlignTags::aligned),
                // this line is not a mistake, we might have overshot in the above line, so we run a bit longer
                new AutoAlignTags(swerveSub).withTimeout(.5), 
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(4),
                // end 2nd note
                new ConditionalCommand(new AutoRotate(swerveSub, 10, 0.08), new AutoRotate(swerveSub, -10, 0.08), this::isBlue),
                Commands.race(
                    new AutoDriveAndTrackNote(swerveSub, 2.5, 0.2),
                    Commands.race(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.15))
                    )
                ),
                //new ConditionalCommand(new AutoRotate(swerveSub, 10, 0.6).withTimeout(1), new AutoRotate(swerveSub, -10, 0.6), this::isBlue),
                new AutoDrive(swerveSub, 1, -0.6).withTimeout(1),
               // new AutoRotate(swerveSub, 45, 0.5)),
                //new AutoAlignTags(swerveSub)
                new ConditionalCommand(new AutoRotate(swerveSub, -11, 0.025), new AutoRotate(swerveSub, 11, 0.025), this::isBlue)
                    .until(AutoAlignTags::speakerAimReady)
                    .withTimeout(4),
                autoAlignSpeaker3.until(autoAlignSpeaker3::aligned),//.until(AutoAlignTags::aligned),
                // this line is not a mistake, we might have overshot in the above line, so we run a bit longer
                new AutoAlignTags(swerveSub).withTimeout(1), 
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(2)
             );

                // new AutoRotate(swerveSub, 90, 0.5),
                //  Commands.race(
                //     new AutoDrive(swerveSub, 3, 0.2),
                //     Commands.race(
                //         new AutoIntake(intakeSub),
                //         new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.3))
                //     )
                // ),
                // new Auto
    }

    
    private MedianFilter zFilter = new MedianFilter(7);
    private boolean closeEnough() {
        if(LimelightHelpers.getTV("limelight-back")){
        var rawZ = LimelightHelpers.getTargetPose3d_CameraSpace(("limelight-back")).getZ();
        var z = zFilter.calculate(rawZ);
        if(z < 1.3){
            return true;
        }
        else{
            return false;
        }
        }
        else{
            return false;
        }

    }

    private boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            if (ally.get() == Alliance.Blue) {
                return true;
            }
            else {
                return false;
            }
        }
        else{
            return false;
        }
    }
}
