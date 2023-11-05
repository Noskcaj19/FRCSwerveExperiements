// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft = new SwerveModule(15,
            14,
            20,
            false,
            false,
            Rotation2d.fromRadians(1.5754), false);
    private SwerveModule frontRight = new SwerveModule(13,
            12,
            19,
            false,
            false,
            Rotation2d.fromRadians(-1.2026), false);
    private SwerveModule backLeft = new SwerveModule(17,
            16,
            21,
            false,
            false,
            Rotation2d.fromRadians(-2.6982), false);
    private SwerveModule backRight = new SwerveModule(11,
            10,
            18,
            false,
            false,
            Rotation2d.fromRadians(2.6952), true);

    private final SlewRateLimiter fwdSpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter strafeSpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DriveConstants.kinematics,
            gyro.getRotation2d().unaryMinus(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                gyro.getRotation2d().unaryMinus(),
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

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d().unaryMinus(),
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
    public void drivePercent(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
        var fwdSpeed = fwdSpeedLimiter.calculate(xPercent) * Constants.DriveConstants.kMaxVelocityMetersPerSecond;

        var strafeSpeed = strafeSpeedLimiter.calculate(yPercent)
                * Constants.DriveConstants.kMaxVelocityMetersPerSecond;

        var rotSpeed = (rotLimiter.calculate(rotPercent) * Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond)
                / 1;

        var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                fwdSpeed, strafeSpeed, rotSpeed, gyro.getRotation2d().unaryMinus())
                        : new ChassisSpeeds(fwdSpeed, strafeSpeed, rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.DriveConstants.kMaxVelocityMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
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
        return gyro.getRotation2d().unaryMinus().getDegrees();
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
}
