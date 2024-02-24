package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleHelper.Souffle;

public class Intake extends SubsystemBase {
    // first stage (wheels)
    VictorSPX intake1 = new VictorSPX(8);
    //  second stage (tubes)
    VictorSPX intake2 = new VictorSPX(9);

    // bed
    TalonSRX rightShoot = new TalonSRX(10);
    TalonSRX leftShoot = new TalonSRX(11);
    TalonSRX feeder = new TalonSRX(12);

    private LaserCan laser = new LaserCan(44);
    // private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(.93551, .002093, 0.000317);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(.93551, .002093);

    public Intake() {

        // intake2.follow(intake1);
        intake2.setInverted(true);
        intake1.setNeutralMode(NeutralMode.Brake);
        intake2.setNeutralMode(NeutralMode.Brake);
        feeder.setNeutralMode(NeutralMode.Brake);
        // feeder.follow(intake1);

        rightShoot.follow(leftShoot);
        rightShoot.setInverted(true);

        leftShoot.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // invert sensor:  Positive Sensor Reading should match Green (blinking) Leds on Talon
        leftShoot.setSensorPhase(true);
        leftShoot.configVoltageCompSaturation(12);
        leftShoot.enableVoltageCompensation(true);

        rightShoot.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // invert sensor:  Positive Sensor Reading should match Green (blinking) Leds on Talon
        rightShoot.setSensorPhase(true);
        rightShoot.configVoltageCompSaturation(12);
        rightShoot.enableVoltageCompensation(true);

        // leftShoot.config_kP(0, 17728);
        // leftShoot.config_kI(0, 0);
        // leftShoot.config_kD(0, 0);
        // leftShoot.config_kF(0, 0);


        var conf = new TalonSRXConfiguration();
        conf.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
        conf.velocityMeasurementWindow = 1;
        leftShoot.configAllSettings(conf);
        rightShoot.configAllSettings(conf);

        // --Very Important--
        rightShoot.setNeutralMode(NeutralMode.Coast);
        leftShoot.setNeutralMode(NeutralMode.Coast);
    }

    private boolean tryIntake = false;
    private boolean shooting;

    public void intake() {
        tryIntake = true;
    }

    public void shootOn() {
        // leftShoot.set(ControlMode.PercentOutput, .5);
        leftShoot.set(TalonSRXControlMode.PercentOutput, ff.calculate(500)/12);
        shooting = true;
        feeder.set(ControlMode.PercentOutput, .5);
    }

    public void intakeOff() {
        tryIntake = false;
        intake1.set(ControlMode.PercentOutput, 0);
        intake2.set(ControlMode.PercentOutput, 0);
    }
    
    public void shootOff() {
        leftShoot.set(ControlMode.PercentOutput, 0);
        shooting = false;
    }

    @Override
    public void periodic() {
        System.out.println(rightShoot.getSelectedSensorVelocity());
        if (tryIntake) {
            LaserCan.Measurement measurement = laser.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                Souffle.set("Debug", "DistanceMM", measurement.distance_mm);
                if (measurement.distance_mm > 250) {
                    intake1.set(ControlMode.PercentOutput, .6);
                    intake2.set(ControlMode.PercentOutput, .7);
                    feeder.set(ControlMode.PercentOutput, 0.1);
                    return;
                }
                if (measurement.distance_mm > 200) {
                    intake1.set(ControlMode.PercentOutput, .3);
                    intake2.set(ControlMode.PercentOutput, .5);
                    feeder.set(ControlMode.PercentOutput, .1);
                    return;
                }
                if (measurement.distance_mm > 160) {
                    intake1.set(ControlMode.PercentOutput, 0);
                    intake2.set(ControlMode.PercentOutput, .45);
                    feeder.set(ControlMode.PercentOutput, .1);
                    return;
                }
            }
        }
        intake1.set(ControlMode.PercentOutput, 0);
        intake2.set(ControlMode.PercentOutput, 0);
        if (!shooting) {
            feeder.set(ControlMode.PercentOutput, 0);
        }
    }
}
