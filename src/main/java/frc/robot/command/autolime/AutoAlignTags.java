package frc.robot.command.autolime;

import java.util.Optional;
import java.util.stream.Stream;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsytems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;

public class AutoAlignTags extends Command {

    private SwerveSubsystem swerveSub;
    private ProfiledPIDController xPID;
    private ProfiledPIDController distancePID;
    private boolean turnOff = false;
    private double backTagID;
    private double frontTagID;
    private int tagChoice;
    // private static double rot;
    // private static double yOff;

    // static double getZontal() {
    //     return (LimelightHelpers.getTX("limelight-back") / 27);
    //     // return (x.getDouble(160)/160)-1;
    //     // horizontal offset
    // }

    final Optional<Pose3d> getSpace() {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-back");

        var target = Stream.of(llresults.targets_Fiducials).filter(f->f.fiducialID == 7).findFirst();
        // var target = Stream.of(LimelightHelpers.getRawFiducials("limelight-back"))
        //     .filter(f -> f.id == 7)
        //     .findFirst();
        if (target.isPresent()) {
            return Optional.of(target.get().getTargetPose_RobotSpace());
        // return (LimelightHelpers.getTargetPose3d_RobotSpace("limelight-back"));
        }
        return Optional.empty();
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }

    public static boolean speakerAimReady() {
        return LimelightHelpers.getTV("limelight-back");
    }

    public AutoAlignTags(SwerveSubsystem swerveSub) {

        // ignore me bbg
        // make tr

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        xPID = new ProfiledPIDController(3*.6, 0.8*.5, .8*.125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3/1.5));
        distancePID = new ProfiledPIDController(3*.6, .8*.5, .8*.125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3/1.5));

        // the robot cant like run into the limelight he needs to be close but not too
        // close omg im gonna die
        distancePID.setGoal(1.5);
        distancePID.setIntegratorRange(-15, 15);
        xPID.setIntegratorRange(-15, 15);
    }

    public boolean aligned(){
        // if (!LimelightHelpers.getTV("limelight-back")) {
        //     return false;
        // }

// var c = new PhotonCamera(null);
// c.getLatestResult().targets[0]

        var target_o = getSpace();
        if (target_o.isEmpty()) {
            return false;
        }
        var target = target_o.get();
        if((target.getZ()< 1.55 && target.getZ() > 1.4) && (Math.abs(target.getX()) < 0.2)){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void initialize() {
        distancePID.reset(1.5);
        xPID.reset(0);

    }

    @Override
    public void execute() {
        

        

        // if (LimelightHelpers.getTV("limelight-back")) {
        //     var id = LimelightHelpers.getFiducialID("limelight-back");
        var target_o = getSpace();
        if (target_o.isPresent()) {
            var target = target_o.get();
            // if (!(id == 7 || id == 4)) { return; }
            // backTagID = LimelightHelpers.getFiducialID("limelight-back");
                        // double xOff = -xPID.calculate(getZontal());
                        var rot = xPID.calculate(target.getX());
                        rot = MathUtil.clamp(rot, -DriveConstants.MaxVelocityMetersPerSecond/5, DriveConstants.MaxVelocityMetersPerSecond/5);
                        // var xOff = 0.0;

                        var df = NetworkTableInstance.getDefault();
                        df.getEntry("/Shuffleboard/Tune/LimeZ").setDouble(target.getZ());
                        double yOff = distancePID.calculate(target.getZ());
                        yOff = MathUtil.clamp(yOff, -DriveConstants.MaxVelocityMetersPerSecond/3.5, DriveConstants.MaxVelocityMetersPerSecond/3.5);
                        df.getEntry("/Shuffleboard/Tune/DistancePID").setDouble(yOff);
                        // figure out how to use an array, which value of the array am i using??

                        // double rot = -distancePID.calculate(getSpace(4));

                        // how do i set a different goal for the distance

                        // System.out.println(getStance());

                        swerveSub.drive(yOff/DriveConstants.MaxVelocityMetersPerSecond, 0/DriveConstants.MaxVelocityMetersPerSecond, rot/DriveConstants.MaxAngularVelocityRadiansPerSecond, false);
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
