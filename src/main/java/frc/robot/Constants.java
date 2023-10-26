// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.sds.ModuleConfiguration;
import frc.robot.sds.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final ModuleConfiguration kModuleType = SdsModuleConfigurations.MK4I_L1;

  public static final class DriveConstants {
    // Distance between left and right wheels
    public static final double kTrackWidthMeters = 0.2921;
    // Distance between front and back wheels
    public static final double kTrackBaseMeters = 0.2921;

    private static final Translation2d m_frontLeftLocation = new Translation2d(kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d m_frontRightLocation = new Translation2d(kTrackBaseMeters, -kTrackWidthMeters);
    private static final Translation2d m_backLeftLocation = new Translation2d(-kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d m_backRightLocation = new Translation2d(-kTrackBaseMeters, -kTrackWidthMeters);

    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double kNeoFreeSpinRpm = 5676;
    public static final double kMaxVelocityMetersPerSecond = (kNeoFreeSpinRpm / 60.0) *
        kModuleType.getDriveReduction() *
        kModuleType.getWheelDiameter() * Math.PI;

    public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
        / Math.hypot(kTrackWidthMeters / 2, kTrackBaseMeters / 2);

  }

  public static final class ModuleConstants {

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 *
        Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    // public static final int kEncoderCPR = 1024;
    // public static final double kWheelDiameterMeters = 0.15;
    // public static final double kDriveEncoderDistancePerPulse =
    // // Assumes the encoders are directly mounted on the wheel shafts
    // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // public static final double kTurningEncoderDistancePerPulse =
    // // Assumes the encoders are on a 1:1 reduction with the module shaft.
    // (2 * Math.PI) / (double) kEncoderCPR;
    public static final double kTurningEncoderDegreesPerPulse = Math
        .toDegrees(2. * Math.PI * kModuleType.getSteerReduction());

    // public static final double kPModuleTurningController = 1;

    // public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
