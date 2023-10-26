// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.ShuffleHelper.ShuffleUtil;
import frc.robot.subsystems.SwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SwerveModule m_frontLeft = new SwerveModule(15,
      14,
      20,
      false,
      false,
      Rotation2d.fromRadians(1.5754));
  private SwerveModule m_frontRight = new SwerveModule(13,
      12,
      19,
      false,
      false,
      Rotation2d.fromRadians(-1.2026));
  private SwerveModule m_backLeft = new SwerveModule(17,
      16,
      21,
      false,
      false,
      Rotation2d.fromRadians(-2.6982));
  private SwerveModule m_backRight = new SwerveModule(11,
      10,
      18,
      false,
      false,
      Rotation2d.fromRadians(2.6952));

  private final SlewRateLimiter fwdSpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter strafeSpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private XboxController controller = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    // var x = new DifferentialDrive(null, null);
    // x.arcadeDrive(kDefaultPeriod, rotation, isAutonomous());
    m_frontLeft.recalEncoders();
    m_frontRight.recalEncoders();
    m_backLeft.recalEncoders();
    m_backRight.recalEncoders();
    gyro.zeroYaw();
  }

  // radians
  double rotation = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    rotation = controller.getRightX();
    var rotationRad = map(rotation, -1, 1, -Math.PI, Math.PI);
    if (controller.getPOV() != -1)
      rotationRad += Units.degreesToRadians(controller.getPOV());
    var rotationRadA = Rotation2d.fromRadians(rotationRad);
    // swerve.m_driveMotor.set(.1);

    // rotation = MathUtil.clamp(rotation + (controller.getRightX() * 0.01570796),
    // -Math.PI, Math.PI);

    // if (controller.getPOV() == 90) {
    // rotation = Math.PI;
    // } else if (controller.getPOV() == 270) {
    // rotation = -Math.PI;
    // }

    // commandedRotationEntry.setDouble(rotationRad);

    var fwdPercent = MathUtil.applyDeadband(-controller.getLeftY(), 0.08);
    var fwdSpeed = fwdSpeedLimiter.calculate(fwdPercent) * Constants.DriveConstants.kMaxVelocityMetersPerSecond;

    var strafePercent = MathUtil.applyDeadband(controller.getLeftX(), 0.08);
    var strafeSpeed = strafeSpeedLimiter.calculate(strafePercent)
        * Constants.DriveConstants.kMaxVelocityMetersPerSecond;

    var rotPercent = MathUtil.applyDeadband(controller.getRightX(), 0.08);
    var rotSpeed = (rotLimiter.calculate(rotPercent) * Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond)
        / 1;

        // example: joystick
    // var stick = new Joystick(1);
    // var swerveModuleStates = DriveConstants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(-stick.getY(), -stick.getX(), stick.getTwist()));
    var fieldRelative = true;
    var swerveModuleStates = DriveConstants.m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                fwdSpeed, strafeSpeed, rotSpeed, gyro.getRotation2d().unaryMinus())
            : new ChassisSpeeds(fwdSpeed, strafeSpeed, rotSpeed));
    // var swerveModuleStates = DriveConstants.m_kinematics.toSwerveModuleStates(
    // new ChassisSpeeds(fwdSpeed, strafeSpeed, rotSpeed));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        Constants.DriveConstants.kMaxVelocityMetersPerSecond);

    /*******
     *******/
    // var flTheta = swerveModuleStates[1].angle.minus(m_frontLeft.getState().angle);
    // var frTheta = swerveModuleStates[0].angle.minus(m_frontRight.getState().angle);
    // var blTheta = swerveModuleStates[3].angle.minus(m_backLeft.getState().angle);
    // var brTheta = swerveModuleStates[2].angle.minus(m_backRight.getState().angle);


    // var errorNumerator = flTheta.getCos() + frTheta.getCos() + blTheta.getCos() + brTheta.getCos();
    // var error = errorNumerator/4;
    /*******
     *******/
    m_frontLeft.setDesiredState(swerveModuleStates[1]);
    m_frontRight.setDesiredState(swerveModuleStates[0]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);
    m_backRight.setDesiredState(swerveModuleStates[2]);

    if (controller.getBackButton()) {
      gyro.zeroYaw();
    }
  }

  double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
