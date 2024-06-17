package frc.robot.command.autolime;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;

public class AutoShootEndless extends Command {

    private Shooter shootSub;
    private Intake intakeSub;

    public AutoShootEndless(Shooter shootSub, Intake intakeSub) {

        addRequirements(shootSub);
        addRequirements(intakeSub);
        this.shootSub = shootSub;
        this.intakeSub = intakeSub;
    }

    @Override
    public void execute() {

        shootSub.shootSpeaker();
        if (shootSub.isToSpeakerSpeed()) {
            intakeSub.feedOn();
        } else {
            intakeSub.feedOff();
        }
    }

}
