package frc.robot.command;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.Arms;

public class ResetClimb extends Command{
    private MedianFilter leftArmFilter = new MedianFilter(3);
    private MedianFilter rightArmFilter = new MedianFilter(3);
    private Arms climbSub;

    public ResetClimb (Arms climbSub) {
        addRequirements(climbSub);
        this.climbSub = climbSub;
    }

    @Override
    public void initialize() {
        leftFinished = false;
        rightFinished = false;
        leftArmFilter.reset();
        rightArmFilter.reset();
        climbSub.rightOverride(.65);
        climbSub.rightOverride(.65);
    }

    boolean leftFinished = false;
    boolean rightFinished = false;

    @Override
    public void execute(){
        if(!leftFinished) {
            var leftStatus = leftArmFilter.calculate(climbSub.getLeftCurrent());
            if(Math.abs(leftStatus) > 8){
                climbSub.leftOverride(0);
                leftFinished = true;
            }
        }
        if(!rightFinished) {
            var rightStatus = rightArmFilter.calculate(climbSub.getRightCurrent());
            if(Math.abs(rightStatus) > 8){
                climbSub.rightOverride(0);
                rightFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSub.leftOverride(0);
        climbSub.rightOverride(0);
    }

    @Override
    public boolean isFinished() {
        return leftFinished && rightFinished;
    }
}
