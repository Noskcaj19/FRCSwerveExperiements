package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsytems.SwerveSubsystem;

//DOES NOT WORK

public class AutoAlignNotes extends Command {

    private SwerveSubsystem swerveSub;
    private ProfiledPIDController horizontalPid;
    private ProfiledPIDController forwardPid;
    private boolean turnOff = false;
    private double backTagID;
    private double frontTagID;
    private int tagChoice;
    private boolean noteDetected;

    public AutoAlignNotes(SwerveSubsystem swerveSub) {

        // ignore me bbg
        // make tr

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        double ku = .4;
        double tu = 1.1;
        double kuh = .3;
        double tuh = 1.6;
        horizontalPid = new ProfiledPIDController(kuh * .6, tuh * .5, tuh * .125,
                // horizontalPid = new ProfiledPIDController(kuh*.6, 0, tuh*.125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        forwardPid = new ProfiledPIDController(ku * .6, tu * .5, tu * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));

        // the robot cant like run into the limelight he needs to be close but not too
        // close omg im gonna die
        // forwardPpid.setGoal();
        forwardPid.setGoal(10);
        horizontalPid.setGoal(10);
        forwardPid.setIntegratorRange(-2, 2);
        horizontalPid.setIntegratorRange(-15, 15);

    }

    public static boolean noteFound() {
        return LimelightHelpers.getTV("limelight-front");
    }

    final double getHorizontalOffset() {
        return (LimelightHelpers.getTX("limelight-front"));
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }

    final double getDistanceOffset() {
        return (LimelightHelpers.getTY("limelight-front"));
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }

//    public boolean aligned(){
//         if (!LimelightHelpers.getTV("limelight-back")) {
//             return false;
//         }
//         if((getSpace().getZ()< 1.55 && getSpace().getZ() > 1.4) && (Math.abs(getSpace().getX()) < 0.2)){
//             return true;
//         }
//         else{
//             return false;
//         }
//    }

    @Override
    public void initialize() {
        // forwardPid.reset(1.5);
        // horizontalPid.reset(0);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-front")) {
            // var id = LimelightHelpers.getFiducialID("limelight-back");
            // if (!(id == 7 || id == 4)) { return; }
            // backTagID = LimelightHelpers.getFiducialID("limelight-back");
            // double xOff = -xPID.calculate(getZontal());
            var yOff = horizontalPid.calculate(getHorizontalOffset());
            yOff = MathUtil.clamp(yOff, -DriveConstants.MaxVelocityMetersPerSecond / 5, DriveConstants.MaxVelocityMetersPerSecond / 5);

            var df = NetworkTableInstance.getDefault();
            df.getEntry("/Shuffleboard/Tune/LimeY").setDouble(getDistanceOffset());
            df.getEntry("/Shuffleboard/Tune/LimeYTan").setDouble(Math.tan(Math.toRadians(getDistanceOffset())));
            double xOff = -forwardPid.calculate(Math.tan(Math.toRadians(getDistanceOffset())));
            xOff = MathUtil.clamp(xOff, -DriveConstants.MaxVelocityMetersPerSecond / 3.5, DriveConstants.MaxVelocityMetersPerSecond / 3.5);
            df.getEntry("/Shuffleboard/Tune/AutoAlignNoteOutputX").setDouble(xOff);
            df.getEntry("/Shuffleboard/Tune/AutoAlignNoteOutputY").setDouble(yOff);
            // figure out how to use an array, which value of the array am i using??

            // double rot = -forwardPpid.calculate(getSpace(4));

            // how do i set a different goal for the distance

            // System.out.println(getStance());

            swerveSub.drive(xOff / DriveConstants.MaxVelocityMetersPerSecond, 0 / DriveConstants.MaxVelocityMetersPerSecond, 0 / DriveConstants.MaxAngularVelocityRadiansPerSecond, false);
            // is x forward and backward??
            // wtf
            // is y forward?
        }

        // else if(LimelightHelpers.getTV("limelight-front")){
        //     if(!LimelightHelpers.getTV("limelight-back")){
        //         swerveSub.drive(0, 0, 0.25, false);
        //     }
        // }
        else {
            swerveSub.drive(0, 0, 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }
}

