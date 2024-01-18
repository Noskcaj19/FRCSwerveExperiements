// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.ShuffleHelper.ShuffleUtil;
// import frc.robot.ShuffleHelper.Test;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // ShuffleUtil.set("null", "null", "null");
    // Stream.of(Test.class.getAnnotations()).forEach(v-> System.out.println(v.annotationType()));
    // Test.class.
    System.out.println(Constants.kModuleType.getWheelDiameter());
    System.out.println(Constants.kModuleType.getDriveReduction());
    RobotBase.startRobot(Robot::new);
  }
}
