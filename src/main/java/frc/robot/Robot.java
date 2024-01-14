// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoStrafeNote;
import frc.robot.subsystems.DriveSubsystem;

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

  // private XboxController controller = new XboxController(0);
  private Joystick stick = new Joystick(0);
  // private Joystick stick2 = new Joystick(1);
  private DriveSubsystem driveSubsystem = new DriveSubsystem();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    var trigger = new JoystickButton(stick, 11);
    trigger.whileTrue(new AutoStrafeNote(driveSubsystem));

    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      var fast = stick.getTrigger();
      var fwdPercent = MathUtil.applyDeadband(-stick.getY(), 0.08) * (fast ? 1 : .5);
      var strafePercent = MathUtil.applyDeadband(stick.getX(), 0.08) * (fast ? 1 : .5);
      var rotPercent = MathUtil.applyDeadband(stick.getTwist(), 0.08) * (fast ? .5 : .15);

      // var b = MathUtil.applyDeadband(stick2.getX(), 0.08) ;
      // var a = MathUtil.applyDeadband(-stick2.getY(), 0.08);
      var a = 0;
      var b = 0 ;

      // var fast = controller.getRightBumper();
      // var fwdPercent = MathUtil.applyDeadband(-controller.getLeftY(), 0.08) * (fast
      // ? 1 : .5);
      // var strafePercent = MathUtil.applyDeadband(controller.getLeftX(), 0.08) *
      // (fast ? 1 : .5);
      // var rotPercent = MathUtil.applyDeadband(controller.getRightX(),
      // 0.08)*(fast?.5 : .15);

      driveSubsystem.drivePercent(fwdPercent, strafePercent, rotPercent, true,a,b );

      // if (controller.getBackButton()) {
      // driveSubsystem.zeroYaw();
      // }
      if (stick.getRawButtonPressed(12)) {
        driveSubsystem.zeroYaw();
      }
      // if (controller.getAButton())
      // driveSubsystem.recalEncoders();
    }, driveSubsystem));
    driveSubsystem.recalEncoders();
  }

  @Override
  public void teleopInit() {
    // var x = new DifferentialDrive(null, null);
    // x.arcadeDrive(kDefaultPeriod, rotation, isAutonomous());

    driveSubsystem.resetEncoders();
  }

  @Override
  public void teleopPeriodic() {
    // swerve.m_driveMotor.set(.1);

    // rotation = MathUtil.clamp(rotation + (controller.getRightX() * 0.01570796),
    // -Math.PI, Math.PI);

    // if (controller.getPOV() == 90) {
    // rotation = Math.PI;
    // } else if (controller.getPOV() == 270) {
    // rotation = -Math.PI;
    // }

    // commandedRotationEntry.setDouble(rotationRad);

    // example: joystick

    // ---
    // var swerveModuleStates = DriveConstants.m_kinematics.toSwerveModuleStates(
    // new ChassisSpeeds(fwdSpeedJ, strafeSpeedJ, rotSpeedJ));

    // var swerveModuleStates =
    // DriveConstants.m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
    // new ChassisSpeeds(MathUtil.applyDeadband(-stick.getY(), 0.1),
    // MathUtil.applyDeadband(stick.getX(), 0.1),
    // MathUtil.applyDeadband(stick.getTwist(), 0.1)),
    // gyro.getRotation2d().unaryMinus()));

    /*******
     *******/

    // swerveModuleStates[1] =
    // m_frontLeft.optimizeModuleState(swerveModuleStates[1]);
    // swerveModuleStates[0]
    // =m_frontRight.optimizeModuleState(swerveModuleStates[0]);
    // swerveModuleStates[3] =m_backLeft.optimizeModuleState(swerveModuleStates[3]);
    // swerveModuleStates[2]
    // =m_backRight.optimizeModuleState(swerveModuleStates[2]);

    // System.out.println("target: " + swerveModuleStates[1].angle.getRadians() + "
    // current " +m_frontLeft.getState().angle.getRadians() + " diff = " +
    // swerveModuleStates[1].angle.minus(m_frontLeft.getState().angle).getRadians());
    // var flTheta = Rotation2d.fromRadians(swerveModuleStates[1].angle.getRadians()
    // % (Math.PI * 2.0))
    // .minus(Rotation2d.fromRadians(m_frontLeft.getState().angle.getRadians() %
    // (Math.PI * 2.0)));
    // var frTheta = Rotation2d.fromRadians(swerveModuleStates[0].angle.getRadians()
    // % (Math.PI * 2.0))
    // .minus(Rotation2d.fromRadians(m_frontRight.getState().angle.getRadians() %
    // (Math.PI * 2.0)));
    // var blTheta = Rotation2d.fromRadians(swerveModuleStates[3].angle.getRadians()
    // % (Math.PI * 2.0))
    // .minus(Rotation2d.fromRadians(m_backLeft.getState().angle.getRadians() %
    // (Math.PI * 2.0)));
    // var brTheta = Rotation2d.fromRadians(swerveModuleStates[2].angle.getRadians()
    // % (Math.PI * 2.0))
    // .minus(Rotation2d.fromRadians(m_backRight.getState().angle.getRadians() %
    // (Math.PI * 2.0)));

    // // linear? cos? cos^3?
    // var errorNumerator = Math.abs(flTheta.getCos()) + Math.abs(frTheta.getCos())
    // + Math.abs(blTheta.getCos())
    // + Math.abs(brTheta.getCos());
    // var error = errorNumerator / 4;

    // swerveModuleStates[0].speedMetersPerSecond *= error;
    // swerveModuleStates[1].speedMetersPerSecond *= error;
    // swerveModuleStates[2].speedMetersPerSecond *= error;
    // swerveModuleStates[3].speedMetersPerSecond *= error;
    /*******
     *******/

    // if (controller.getBackButton()) {
    // gyro.zeroYaw();
    // }
  }

  @Override
  public void autonomousInit() {
    getAutonomousCommand().schedule();
  }

  double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  @Override
  public void simulationPeriodic() {
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

  public Command getAutonomousCommand2() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1/2, 1/2), new Translation2d(2/2, -1/2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3/2, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kinematics,

        // Position controllers
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drivePercent(0, 0, 0, false,0,0));
  }
}
