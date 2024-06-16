package frc.robot.subsytems;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;


//add motor channel numbers later
public class SwerveSubsystem extends SubsystemBase {

        private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);

        private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

        private final SwerveModule fLSwerve = new SwerveModule(15, 14, 20, true, true, -0.3666);
        private final SwerveModule fRSwerve = new SwerveModule(13, 12, 19, true, true, -0.49316);
        private final SwerveModule bLSwerve = new SwerveModule(17, 16, 21, true, true, 0.3317);
        private final SwerveModule bRSwerve = new SwerveModule(11, 10, 18, true, true, -0.0656);

        private final SwerveModule[] modules = {
                        fLSwerve,
                        fRSwerve,
                        bLSwerve,
                        bRSwerve
        };

        private static AHRS gyro = new AHRS(SPI.Port.kMXP);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        private final Field2d m_field = new Field2d();

        private Rotation2d fodOffset = new Rotation2d();

        SwerveDrivePoseEstimator ometry = new SwerveDrivePoseEstimator(
                        kinematics,
                        gyro.getRotation2d(),
                        getModulePositions(),
                        new Pose2d(1.5, 5.55, Rotation2d.fromDegrees(0)),
                        VecBuilder.fill(0.1, 0.1, 0.4),
                        VecBuilder.fill(0.9, 0.9, 0.9));

        public void setFodOffset() {
                fodOffset = gyro.getRotation2d();
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                drive(xPercent, yPercent, rotPercent, fieldRelative, 0, 0);
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative, double a,
                        double b) {

                var xSpeed = xRateLimiter.calculate(xPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                ChassisSpeeds chasSpeed = fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                ometry.getEstimatedPosition().getRotation().minus(fodOffset))
                                : new ChassisSpeeds(xSpeed, ySpeed, rot);

                // TODO: DEFINE MAX SPEED
                var swerveModuleStates2 = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chasSpeed, 0.2),
                                new Translation2d(DriveConstants.kTrackBaseMeters * a * 1.5,
                                                DriveConstants.kTrackWidthMeters * b * 1.5));

                driveStates(swerveModuleStates2);

        }

        public void zeroYaw() {
                gyro.zeroYaw();
        }

        public boolean hasSeenMegatag() {
                return seenMT;
        }
        private boolean seenMT = false;
        void mtVision() {
                boolean useMegaTag2 = false; // set to false to use MegaTag1
                boolean doRejectUpdate = false;
                if (useMegaTag2 == false) {
                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue("limelight-back");

                        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                                if (mt1.rawFiducials[0] == null) {
                                        return;
                                }
                                if (mt1.rawFiducials[0].ambiguity > .7) {
                                        doRejectUpdate = true;
                                }
                                if (mt1.rawFiducials[0].distToCamera > 3) {
                                        doRejectUpdate = true;
                                }
                        }
                        if (mt1.tagCount == 0) {
                                doRejectUpdate = true;
                        }

                        if (!doRejectUpdate) {
                                seenMT = true;
                                ometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                ometry.addVisionMeasurement(
                                                mt1.pose,
                                                mt1.timestampSeconds);
                        }
                } else if (useMegaTag2 == true) {
                        LimelightHelpers.SetRobotOrientation("limelight-back",
                                        ometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
                        if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per
                                                            // second, ignore vision updates
                        {
                                doRejectUpdate = true;
                        }
                        if (mt2.tagCount == 0) {
                                doRejectUpdate = true;
                        }
                        if (!doRejectUpdate) {
                                seenMT = true;
                                ometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                ometry.addVisionMeasurement(
                                                mt2.pose,
                                                mt2.timestampSeconds);

                        }
                }
        }


        public CircularBuffer<PathPoint> history = new CircularBuffer<>(50*10);
        @Override
        public void periodic() {
                mtVision();

                ometry.update(gyro.getRotation2d(), getModulePositions());


                if (recordEntry.getBoolean(false)) {
                        System.err.println("Recording");
                        var velocity = Math.hypot(Math.pow(getSpeeds().vxMetersPerSecond, 2) , Math.pow(getSpeeds().vyMetersPerSecond, 2));
                        // if (velocity < .2) {} 
                        velocity = MathUtil.clamp(velocity, 0.2, Double.POSITIVE_INFINITY);
                        var rot = new RotationTarget(0, ometry.getEstimatedPosition().getRotation(), true);
                        history.addLast(new PathPoint(ometry.getEstimatedPosition().getTranslation(), rot, new PathConstraints(velocity, 3, Math.PI, Math.PI)));
                        if (history.size() == 50*10) {
                                recordEntry.setBoolean(false);
                        }
                }
                

                m_field.setRobotPose(getPose());
        }

        public Pose2d getPose() {
                return ometry.getEstimatedPosition();
        }

        public double getYaw() {
                return gyro.getYaw();
        }

        public void resetOmetry(Pose2d pose) {
                ometry.resetPosition(
                                gyro.getRotation2d(),
                                getModulePositions(),
                                                pose);
        }

        private SwerveModulePosition[] getModulePositions() {
                return Stream.of(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
        }

        public SwerveModuleState[] getModuleStates() {
                return Stream.of(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        }

        public ChassisSpeeds getSpeeds() {
                return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        }

        public void driveStates(SwerveModuleState[] swerveModuleStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                                Constants.DriveConstants.MaxVelocityMetersPerSecond);

                SwerveModuleState[] optimizedSwerveModuleStates = {
                                fLSwerve.optimizeModuleState(swerveModuleStates[0]),
                                fRSwerve.optimizeModuleState(swerveModuleStates[1]),
                                bLSwerve.optimizeModuleState(swerveModuleStates[2]),
                                bRSwerve.optimizeModuleState(swerveModuleStates[3]),
                };

                fLSwerve.setDesiredState(optimizedSwerveModuleStates[0]);
                fRSwerve.setDesiredState(optimizedSwerveModuleStates[1]);
                bLSwerve.setDesiredState(optimizedSwerveModuleStates[2]);
                bRSwerve.setDesiredState(optimizedSwerveModuleStates[3]);
        }

        final NetworkTableEntry recordEntry;
        public SwerveSubsystem() {
                Shuffleboard.getTab("Drive").add(m_field);

                recordEntry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive").getEntry("Record");
                recordEntry.setBoolean(false);

                // Configure AutoBuilder last
                AutoBuilder.configureHolonomic(
                                this::getPose, // Robot pose supplier
                                this::resetOmetry, // Method to reset odometry (will be called if your auto has a
                                                   // starting pose)
                                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                (ChassisSpeeds speeds) -> {
                                        var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                                                        ChassisSpeeds.discretize(speeds, .02));
                                        driveStates(swerveModuleStates);
                                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                                                                 // in your
                                                                 // Constants class
                                                // good constants
                                                // new PIDConstants(5, 0.01, .0), // Translation PID constants
                                                // new PIDConstants(7, 0.01, 0.0), // Rotation PID constants
                                                new PIDConstants(8, 0.01, .0), // Translation PID constants
                                                new PIDConstants(8, 0.01, 0.0), // Rotation PID constants
                                                3.0, // Max module speed, in m/s
                                                0.3, // Drive base radius in meters. Distance from robot center to
                                                     // furthest module.
                                                new ReplanningConfig(true, true) // Default path replanning config. See the API
                                                                       // for the options here
                                ),
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );
        }

        public void runVolts(Measure<Voltage> voltage) {
                fLSwerve.runVolts(voltage);
                fRSwerve.runVolts(voltage);
                bLSwerve.runVolts(voltage);
                bRSwerve.runVolts(voltage);
        }
}
