// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_integratedTurningEncoder;
  private final CANCoder m_absoluteEncoder;

  private final PIDController m_drivePIDController = new PIDController(.01, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
       0.064059,0,0.0069,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int turningEncoderId,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      float magnetOffset) {
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // #region Motor controller setup
    m_driveMotor.setInverted(driveEncoderReversed);
    m_turningMotor.setInverted(turningEncoderReversed);

    m_turningMotor.setSmartCurrentLimit(20);
    m_driveMotor.setSmartCurrentLimit(80);

    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
    // Set neutral mode to brake
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // #endregion

    m_driveEncoder = m_driveMotor.getEncoder();
    var positionConversionFactor = Math.PI * Constants.kModuleType.getWheelDiameter()
        * Constants.kModuleType.getDriveReduction();
    m_driveEncoder.setPositionConversionFactor(positionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

    m_integratedTurningEncoder = m_turningMotor.getEncoder();
    Shuffleboard.getTab("Debug").addDouble("Integrated encoder", () -> m_integratedTurningEncoder.getPosition());
    // m_integratedTurningEncoder.setPositionConversionFactor(1);
    // m_integratedTurningEncoder.setVelocityConversionFactor(1 / 60);

    m_integratedTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDegreesPerPulse);
    m_integratedTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderDegreesPerPulse / 60);

    m_absoluteEncoder = new CANCoder(turningEncoderId);

    var config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.magnetOffsetDegrees = magnetOffset;
    config.sensorDirection = turningEncoderReversed;

    m_absoluteEncoder.configAllSettings(config, 250);
    m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    Shuffleboard.getTab("Debug").addDouble("Turn Output Raw", () -> m_turningMotor.get());
    Shuffleboard.getTab("Debug").addDouble("Drive Output Raw", () -> m_driveMotor.get());
    Shuffleboard.getTab("Debug")
        .addDouble("Measured rotation",
            () -> m_absoluteEncoder.getAbsolutePosition())
        .withWidget(BuiltInWidgets.kGyro);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_absoluteEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_absoluteEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state = SwerveModuleState.optimize(desiredState,
    //     new Rotation2d(m_absoluteEncoder.getAbsolutePosition()));
    SwerveModuleState state= desiredState;

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(map(m_absoluteEncoder.getAbsolutePosition(), -180, 180, -Math.PI, Math.PI),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    // m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_absoluteEncoder.setPosition(0);
  }
}
