// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private SwerveModule swerve;
  private XboxController controller;
  private GenericEntry commandedRotationEntry;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    swerve = new SwerveModule(12,
        13,
        3,
        false,
        false,
        294);

    controller = new XboxController(0);

  

    commandedRotationEntry = Shuffleboard.getTab("Debug")
        .add("Commanded rotation", 0)
        .withWidget(BuiltInWidgets.kGyro).getEntry();
  }

  // radians
  double rotation = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    rotation = controller.getRightX();

    // rotation = MathUtil.clamp(rotation + (controller.getRightX() * 0.01570796),
    // -Math.PI, Math.PI);

    // if (controller.getPOV() == 90) {
    // rotation = Math.PI;
    // } else if (controller.getPOV() == 270) {
    // rotation = -Math.PI;
    // }

    commandedRotationEntry.setDouble(map(rotation, -1, 1,-Math.PI, Math.PI));

    // var drivePercentage = controller.getLeftY();
    System.out.println(rotation);

    // var driveSpeed = drivePercentage * Constants.DriveConstants.kMaxVelocityMetersPerSecond;
    swerve.setDesiredState(
        new SwerveModuleState(0, new Rotation2d(map(rotation, -1, 1, -Math.PI, Math.PI))));
  }

  double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
