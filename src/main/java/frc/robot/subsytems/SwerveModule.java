package frc.robot.subsytems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class SwerveModule {

        // add motor model later

        // defines motors for a singular module

        public final CANSparkMax driveMotor;
        private final CANSparkMax turningMotor;

        // add encoder model later

        // defines encoders for a singular module

        private final RelativeEncoder driveEncoder;
        private final RelativeEncoder turningEncoder;
        private final CANcoder absoluteEncoder;

        // pid drive controller somehow
        // add tuning values

        private final PIDController drivePIDController = new PIDController(.1, 0, 0);

        // i i i ffffffffffffffffffff
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.18359, 3.0413);
        // pid turning trapezoidal

        // private final PIDController turningPIDController = new PIDController(1.0 /
        // (Math.PI / 2), 0, .0);

        // add values later

        // private final SimpleMotorFeedforward driveFeedforward = new
        // SimpleMotorFeedforward(s, v);
        private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.13943, 0.39686, 0.015295);

        private SparkPIDController pidController;

        private final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(
                        Float.POSITIVE_INFINITY,
                        200 * 2);

        public double getAbsRad() {
                return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        }

        public SwerveModule(
                        int driveMotorID,
                        int turningMotorID,
                        int turningEncoderID,
                        boolean driveMotorInverted,
                        boolean turningMotorInverted,
                        double magnetOffset) {
                // add motor name
                // makes it so you can define motor channels for the modules in subsystem
                driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
                turningMotor.restoreFactoryDefaults();

                driveMotor.setInverted(driveMotorInverted);
                turningMotor.setInverted(turningMotorInverted);

                // add encoder name
                // makes it so you can define encoder channels for the modules in subsystem

                absoluteEncoder = new CANcoder(turningEncoderID);

                // turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                var config = new MagnetSensorConfigs();
                config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

                // absoluteEncoder.configAllSettings(config, 250);
                // absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100,
                // 250);
                config.MagnetOffset = magnetOffset;
                // config.MagnetOffset = 0;

                absoluteEncoder.getConfigurator().apply(config);
                absoluteEncoder.getAbsolutePosition().setUpdateFrequency(100, 250);

                // #region Motor controller setup
                // driveMotor.setInverted(driveMotorInverted);
                // turningMotor.setInverted(turningMotorInverted);

                turningMotor.setSmartCurrentLimit(20);
                turningMotor.setOpenLoopRampRate(0.05);
                driveMotor.setSmartCurrentLimit(40);
                // driveMotor.setOpenLoopRampRate(0.075);

                driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
                driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
                // Set neutral mode
                driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

                turningMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
                turningMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                turningMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 50);
                // Set neutral mode
                turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
                // #endregion

                driveEncoder = driveMotor.getEncoder();
                // TODO: FIX CONSTANTS
                double positionConversionFactor = Math.PI * Constants.ModuleType.getWheelDiameter()
                                * Constants.ModuleType.getDriveReduction();
                driveEncoder.setPositionConversionFactor(positionConversionFactor);
                driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

                turningEncoder = turningMotor.getEncoder();

                turningEncoder
                                .setPositionConversionFactor(Math
                                                .toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse));
                turningEncoder
                                .setVelocityConversionFactor(
                                                Math.toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse)
                                                                / 60);
                turningEncoder.setPosition(getAbsRad());

                // turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                pidController = turningMotor.getPIDController();
                pidController.setP(1.0);
                pidController.setI(0.0);
                pidController.setD(0.1);
                // pidController.setFF(1.534);
                pidController.setOutputRange(-1, 1);

                // Shuffleboard.getTab("Debug").addDouble("Turn Output Raw", () ->
                // m_turningMotor.get());
                // Shuffleboard.getTab("Debug").addDouble("Drive Output Raw", () ->
                // m_driveMotor.get());
                Shuffleboard.getTab("Debug")
                                .addDouble("Measured Abs rotation" + turningEncoderID,
                                                () -> getAbsRad());
                // Shuffleboard.getTab("Debug").addDouble("Integrated encoder", () ->
                // m_integratedTurningEncoder.getPosition());
        }

        public SwerveModuleState getState() {
                return new SwerveModuleState(
                                driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
        }

        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
        }

        public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                // SwerveModuleState state = optimizeModuleState(desiredState);
                SwerveModuleState state = desiredState;

                var fff = feedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                driveMotor.setVoltage(
                                ((state.speedMetersPerSecond / Constants.DriveConstants.MaxVelocityMetersPerSecond)
                                                * 12) + fff);

                pidController.setReference(state.angle.getRadians(), ControlType.kPosition);
        }

        public SwerveModuleState optimizeModuleState(SwerveModuleState rawState) {
                var optimizedState = SwerveModuleState.optimize(rawState,
                                Rotation2d.fromRadians(turningEncoder.getPosition()));

                double currentAngleRadiansMod = turningEncoder.getPosition() % (2.0 * Math.PI);
                if (currentAngleRadiansMod < 0.0) {
                        currentAngleRadiansMod += 2.0 * Math.PI;
                }

                // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
                // that
                double adjustedReferenceAngleRadians = optimizedState.angle.getRadians() + turningEncoder.getPosition()
                                - currentAngleRadiansMod;
                if (optimizedState.angle.getRadians() - currentAngleRadiansMod > Math.PI) {
                        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
                } else if (optimizedState.angle.getRadians() - currentAngleRadiansMod < -Math.PI) {
                        adjustedReferenceAngleRadians += 2.0 * Math.PI;
                }

                // pidController.setReference(0,ControlType.kPosition);
                // pidController.setReference(state.angle.getRadians(), ControlType.kPosition);

                return new SwerveModuleState(optimizedState.speedMetersPerSecond,
                                Rotation2d.fromRadians(adjustedReferenceAngleRadians));
        }

        public void resetEncoders() {
                driveEncoder.setPosition(0);
        }

        public void runVolts(Measure<Voltage> voltage) {
                driveMotor.setVoltage(voltage.magnitude());
                pidController.setReference(0, ControlType.kPosition);
        }
}

// but im a CounterClockwise_Positive
// im a weirdoOoooo
// what the hell am i doing here
// i dont belong here
// //oOoO
// shEEEeeeees
// rUnning out of TIIIMmeeee
// shEEessss
// rUUning out of TimETIMETIMETIMEEEEEEEEEeeeee
