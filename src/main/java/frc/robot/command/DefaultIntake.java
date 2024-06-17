package frc.robot.command;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;

public class DefaultIntake extends Command {

    private final CommandJoystick primaryController;
    private final CommandXboxController secondaryController;
    Intake mouth;
    Shooter shooterSub;

    public DefaultIntake(Intake mouth, XboxController secondaryControllerA, Shooter shooterSub, Joystick primaryControllerA) {

        addRequirements(mouth);
        this.secondaryController = new CommandXboxController(secondaryControllerA.getPort());
        this.primaryController = new CommandJoystick(primaryControllerA.getPort());
        this.mouth = mouth;
        this.shooterSub = shooterSub;

        var mouthHasNoteTriggerRaw = new Trigger(() -> mouth.hasNote());
        var mouthHasNoteTrigger = mouthHasNoteTriggerRaw.debounce(1, DebounceType.kFalling);

        mouthHasNoteTrigger.onTrue(rumble(primaryController.getHID(), .5, .25));

        secondaryController.leftBumper().or(primaryController.button(7))
                .onTrue(Commands.runOnce(mouth::smartIntake))
                .onFalse(Commands.runOnce(mouth::stopSmIntake));
    }

    public Command rumble(GenericHID controller, double intensity, double duration) {
        // return Commands.sequence(
        // new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble,
        // intensity)),
        // new WaitCommand(duration),
        // new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
        return Commands.startEnd(
                        () -> {
                            System.err.println("Rumble");
                            controller.setRumble(RumbleType.kBothRumble, intensity);
                        },
                        () -> controller.setRumble(RumbleType.kBothRumble, 0))
                .withTimeout(duration);
    }

    // comment out which control method the drivers dont want
    // private final XboxController primaryController = new XboxController(0);
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        // if (true) {
        // mouth.printshtuff();
        // }

        // if (secondaryController.getLeftBumperPressed() ||
        // primaryController.getRawButtonPressed(7)) {
        // mouth.smartIntake();
        // }
        // if (secondaryController.getLeftBumperReleased() ||
        // primaryController.getRawButtonReleased(7)) {
        // mouth.stopSmIntake();
        // }

        // if (primaryJoystick.getRawButtonPressed(12)) {
        // mouth.smartIntake();
        // // System.out.println(mouth.get());
        // } else if (primaryJoystick.getRawButtonReleased(12)) {
        // mouth.stopSmIntake();
        // // System.out.println(mouth.get());
        // }

        // System.out.println(mouth.getTaking());

        // if (secondaryController.getRightBumperPressed()) {
        // mouth.feedOn();
        // }
        // if (secondaryController.getRightBumperReleased()) {
        // mouth.feedOff();
        // shooterSub.turnOff();
        // }
        // if (primaryJoystick.getRawButton(4)) {
        // mouth.sing();
        // }
        // if (primaryJoystick.getRawButtonReleased(4)) {
        // mouth.intakeOff();
        // }
    }
}