package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // first stage (wheels)
    VictorSPX intake1 = new VictorSPX(8);
    //  second stage (tubes)
    VictorSPX intake2 = new VictorSPX(9);

    // bed
    TalonSRX rightShoot = new TalonSRX(10);
    TalonSRX leftShoot = new TalonSRX(11);
    TalonSRX feeder = new TalonSRX(12);

    public Intake() {
        intake2.follow(intake1);
        intake2.setInverted(true);
        feeder.follow(intake1);

        rightShoot.follow(leftShoot);
        rightShoot.setInverted(true);

        
        rightShoot.setNeutralMode(NeutralMode.Coast);
        leftShoot.setNeutralMode(NeutralMode.Coast);
    }

    public void go(double speed) {
        intake1.set(ControlMode.PercentOutput, speed);

        leftShoot.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        intake1.set(ControlMode.PercentOutput, 0);

        leftShoot.set(ControlMode.PercentOutput, 0);
    }
}
