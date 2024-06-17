package frc.robot.command.autolime;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Intake;

public class AutoIntakeEndless extends Command {

    private Intake intakeSub;

    public AutoIntakeEndless(Intake intakeSub) {
        this.intakeSub = intakeSub;
    }

    @Override
    public void execute() {
        intakeSub.smartIntake();
    }

}