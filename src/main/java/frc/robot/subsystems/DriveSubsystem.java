// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // practice offsets
    // -0.115
    // 0.051
    // -0.234
    // -0.287

    private SwerveModule frontLeft = new SwerveModule(15,
            14,
            20,
            false,
            true,
            -0.137, false);
    private SwerveModule frontRight = new SwerveModule(13,
            12,
            19,
            false,
            true,
            0, false);
    private SwerveModule backLeft = new SwerveModule(17,
            16,
            21,
            false,
            true,
            0.172, false);
    private SwerveModule backRight = new SwerveModule(11,
            10,
            18,
            false,
            true,
            -0.429, true);

    private final SlewRateLimiter fwdSpeedLimiter = new SlewRateLimiter(1.25);
    private final SlewRateLimiter strafeSpeedLimiter = new SlewRateLimiter(1.25);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DriveConstants.kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    private LaserCan lc = new LaserCan(29);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        try {
            lc.setRangingMode(RangingMode.SHORT);
            lc.setTimingBudget(TimingBudget.TIMING_BUDGET_50MS);
            lc.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds) -> {
                    var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                            ChassisSpeeds.discretize(speeds, .02));
                    driveStates(swerveModuleStates);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(9.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(9, 0.0, 0.0), // Rotation PID constants
                        1.5, // Max module speed, in m/s
                        0.3, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
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

    @Override
    public void periodic() {
        // System.out.printf("r: %.2f p: %.2f y: %.2f\n", gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
        // Update the odometry in the periodic block
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drivePercent(double xPercent, double yPercent, double rotPercent, boolean fieldRelative, double a,
            double b) {
        var fwdSpeed = fwdSpeedLimiter.calculate(xPercent) * Constants.DriveConstants.kMaxVelocityMetersPerSecond;
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            final var offset = 100;
            if (measurement.distance_mm < 500 + offset) {
                fwdSpeed *= MathUtil.clamp((measurement.distance_mm - offset) / 500, 0, 1);
            }
        }
        // var fwdSpeed
        // System.out.printf("p: %.2f f: %.2f\n", xPercent, fwdSpeed );

        var strafeSpeed = strafeSpeedLimiter.calculate(yPercent)
                * Constants.DriveConstants.kMaxVelocityMetersPerSecond;

        var rotSpeed = (rotLimiter.calculate(rotPercent) * Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond)
                / 1;

        var chasisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        fwdSpeed, strafeSpeed, rotSpeed,
                        gyro.getRotation2d().plus(Rotation2d.fromDegrees(0)))
                : new ChassisSpeeds(fwdSpeed, strafeSpeed, rotSpeed);

        var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(chasisSpeeds, .02),
                new Translation2d(DriveConstants.kTrackBaseMeters * a * 2.5,
                        DriveConstants.kTrackWidthMeters * b * 2.5));
        // : new ChassisSpeeds(fwdSpeed, strafeSpeed, rotSpeed) );
        driveStates(swerveModuleStates);
    }

    public void driveStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.DriveConstants.kMaxVelocityMetersPerSecond);

        SwerveModuleState[] optimizedSwerveModuleStates = {
                frontLeft.optimizeModuleState(swerveModuleStates[0]),
                frontRight.optimizeModuleState(swerveModuleStates[1]),
                backLeft.optimizeModuleState(swerveModuleStates[2]),
                backRight.optimizeModuleState(swerveModuleStates[3]),
        };

        // var flTheta = swerveModuleStates[0].angle.minus()
        var flTheta = optimizedSwerveModuleStates[0].angle.minus(frontLeft.getState().angle);
        var frTheta = optimizedSwerveModuleStates[1].angle.minus(frontRight.getState().angle);
        var blTheta = optimizedSwerveModuleStates[2].angle.minus(backLeft.getState().angle);
        var brTheta = optimizedSwerveModuleStates[3].angle.minus(backRight.getState().angle);

        var errorNumerator = Math.abs(flTheta.getCos()) + Math.abs(frTheta.getCos())
                + Math.abs(blTheta.getCos())
                + Math.abs(brTheta.getCos());
        var error = errorNumerator / 4;

        optimizedSwerveModuleStates[0].speedMetersPerSecond *= error;
        optimizedSwerveModuleStates[1].speedMetersPerSecond *= error;
        optimizedSwerveModuleStates[2].speedMetersPerSecond *= error;
        optimizedSwerveModuleStates[3].speedMetersPerSecond *= error;

        frontLeft.setDesiredState(optimizedSwerveModuleStates[0]);
        frontRight.setDesiredState(optimizedSwerveModuleStates[1]);
        backLeft.setDesiredState(optimizedSwerveModuleStates[2]);
        backRight.setDesiredState(optimizedSwerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxVelocityMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState(),
        };
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition(),
        };
        return positions;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void recalEncoders() {
        frontLeft.recalEncoders();
        frontRight.recalEncoders();
        backLeft.recalEncoders();
        backRight.recalEncoders();
        gyro.zeroYaw();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * -1.0;
    }

    public void zeroYaw() {
        gyro.zeroYaw();
    }

    public void runVolts(Measure<Voltage> voltage) {
        frontLeft.runVolts(voltage);
        frontRight.runVolts(voltage);
        backLeft.runVolts(voltage);
        backRight.runVolts(voltage);
    }
}
