package frc.robot.command.autolime;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;

public class AutoShootSmart extends SequentialCommandGroup{

    Shooter shooterSub;
    Intake intakeSub;

   public AutoShootSmart(Shooter shooterSub, Intake intakeSub) {

        addRequirements(shooterSub);
        addRequirements(intakeSub);
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        addCommands(
            new AutoShootEndless(shooterSub, intakeSub).until(intakeSub::doesntHaveNote), 
            new WaitCommand(.2),
            new InstantCommand(()->{
                intakeSub.feedOff();
                shooterSub.turnOff();
            }, intakeSub, shooterSub));
   }
}