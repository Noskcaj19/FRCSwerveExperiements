package frc.robot.commands.auto.command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrive extends Command {
    private DriveSubsystem ds;
    private Translation2d startingPosition;
    private double targetDist;
    private double speed;

    public AutoDrive(DriveSubsystem ds, double speedPercent, double targetDist) {
        this.targetDist = targetDist;
        this.speed = speedPercent;
        this.ds = ds;
    }

    @Override
    public void initialize() {
        startingPosition = ds.getPose().getTranslation();
    }

    @Override
    public void execute() {
        ds.drivePercent(speed, 0, 0, false, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        ds.drivePercent(0, 0, 0, false, 0, 0);
    }

    @Override
    public boolean isFinished() {
        var dist = ds.getPose().getTranslation().getDistance(startingPosition);

        return (dist > targetDist);
    
    }
    
}
