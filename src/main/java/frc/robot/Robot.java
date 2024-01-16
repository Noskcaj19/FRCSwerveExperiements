// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.wpilibj.XboxController;
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
  private NetworkTablesController stick = new NetworkTablesController(0);
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
    // CameraServer.startAutomaticCapture();
    // CameraServer.startAutomaticCapture();

    var trigger = new JoystickButton(stick, 11);
    trigger.whileTrue(new AutoStrafeNote(driveSubsystem));

    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      var fast = stick.getRightBumper();
      var fwdPercent = MathUtil.applyDeadband(-stick.getRightY(), 0.08) * (fast ? 1 : .5);
      var strafePercent = MathUtil.applyDeadband(stick.getRightX(), 0.08) * (fast ? 1 : .5);
      var rotPercent = MathUtil.applyDeadband(stick.getLeftX(), 0.08) * (fast ? .5 : .15);

      // var b = MathUtil.applyDeadband(stick2.getX(), 0.08) ;
      // var a = MathUtil.applyDeadband(-stick2.getY(), 0.08);
      var pov = stick.getPOV();
      var a = 0;
      var b = 0 ;
      if (pov == 45) {
        a = -1;
        b = 1;
      } else if (pov == 45 + 90) {
        a = -1;
        b = -1;
      } else if (pov == 45 + 90 + 90) {
        a = 1;
        b = -1;
      } else if (pov == 45 + 90 + 90 + 90) {
        a = 1;
        b = 1;
      }

      // var fast = controller.getRightBumper();
      // var fwdPercent = MathUtil.applyDeadband(-controller.getLeftY(), 0.08) * (fast
      // ? 1 : .5);
      // var strafePercent = MathUtil.applyDeadband(controller.getLeftX(), 0.08) *
      // (fast ? 1 : .5);
      // var rotPercent = MathUtil.applyDeadband(controller.getRightX(),
      // 0.08)*(fast?.5 : .15);

      if (stick.getRawButton(2)) {
        strafePercent = 0;
      }
      // var mod = (stick.getRawAxis(3) + 1) / 2;
      // fwdPercent *= mod; 
      // strafePercent *= mod;
      // rotPercent *= mod;
      if (fwdPercent!=0.0) {

      System.err.println(fwdPercent);
      }
      driveSubsystem.drivePercent(fwdPercent, strafePercent, rotPercent,!stick.getAButton(),a,b );

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
        List.of(new Translation2d(1, 1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
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
