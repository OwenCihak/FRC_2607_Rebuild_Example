// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Encoder {

    private final CANcoder canCoder;
    private final PhoenixPIDController pid;

    public Encoder(int canId, double P, double I, double D){
        canCoder = new CANcoder(canId);
        pid = new PhoenixPIDController(P, I, D);
    }

    public CANcoder getCANCoder(){
        return canCoder;
    }

    public PhoenixPIDController getPIDController(){
        return pid;
    }

    public double getCanCoderPos(){
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getTargetPos(){
        return pid.getSetpoint();
    }

    public double getOutput(double target){
        return pid.calculate(getCanCoderPos(), target, Timer.getFPGATimestamp());
    } 

    public void logEncoderMetrics(String ntKey){
        RobotLogger.logDouble(ntKey + "Encoder/TargetPos", getTargetPos());
        RobotLogger.logDouble(ntKey + "Encoder/CanCoderPos", getCanCoderPos());
    }
}
