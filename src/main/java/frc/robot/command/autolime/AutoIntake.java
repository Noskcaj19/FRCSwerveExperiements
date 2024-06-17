package frc.robot.command.autolime;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Intake;

public class AutoIntake extends Command {

    private Intake intakeSub;

    public AutoIntake(Intake intakeSub) {
        this.intakeSub = intakeSub;
    }

    @Override
    public void execute() {
        intakeSub.smartIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stopSmIntake();
    }
}
