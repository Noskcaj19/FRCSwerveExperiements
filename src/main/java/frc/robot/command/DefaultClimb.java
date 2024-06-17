package frc.robot.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Arms;

public class DefaultClimb extends Command {

    boolean status = false;
    private Joystick joy;
    private Arms climbSub;

    public DefaultClimb(Joystick inJoy, Arms climbSub) {
        addRequirements(climbSub);
        this.climbSub = climbSub;
        this.joy = inJoy;
    }

    @Override
    public void execute() {

        // double extendController;
        // if (secondaryController.getRightY() < 0.03 && secondaryController.getRightY()
        // > -0.03) {
        // extendController = 0;
        // } else {
        // extendController = secondaryController.getRightY() * 2;
        // }

        // var extendSet = -extendController + -clawSystem.getExtendSetPoint();
        // climbSub.set(extendSet);

        // if (joy.getPOV() == 0) {
        // status = !status;
        // climbSub.upDown(status);
        // } else if (joy.getPOV() == 180) {
        // climbSub.upDown(status);
        // } else {
        // climbSub.turnOff();
        // }

        // if (joy.getRawButtonPressed(5)) {
        // status = !status;
        // climbSub.upDown(status);
        // } else if (joy.getRawButton(6)) {
        // climbSub.upDown(status);
        // } else {
        // climbSub.turnOff();
        // }

        if (joy.getPOV() == 180) {
            climbSub.armsUp();
        } else if (joy.getPOV() == 0) {
            climbSub.armsDown();
        } else {
            climbSub.turnOff();
        }
    }

}
