package frc.robot.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsytems.SwerveSubsystem;

public class StopCommand extends InstantCommand {
    private SwerveSubsystem ds;

    public StopCommand(SwerveSubsystem ds) {
        this.ds = ds;
    }

    @Override
    public void execute() {
        ds.drive(0, 0, 0, false);
    }
}